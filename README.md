# wrench_filter

`geometry_msgs/WrenchStamped`（力覚センサの値）向けのROS 2（Humble）パッケージです。次の2つのノードを提供します。

| ノード | 種類 | 役割 |
| --- | --- | --- |
| `wrench_filter::WrenchFilter` | component（`wrench_filter_node`でも起動可） | 6軸それぞれにIIRローパス（Butterworthなど）をかける。NaN/Infのサンプルは捨てる |
| `wrench_filter::WrenchWatchdog` | component（`wrench_watchdog_node`でも起動可） | 力フィードバック系のros2_controlコントローラの入力を監視する。入力が無い・古い・NaN/Infなら、位置を保持するコントローラに切り替える |

`WrenchFilter`は、[quantumbeam/powder_grinding_ros2](https://github.com/quantumbeam/powder_grinding_ros2)の`grinding_force_torque::WrenchFilter`（コミット344d0cc、MIT）を移植したものです。

## WrenchFilter

| パラメータ | 型 / 単位 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `input_topic` | string | `wrench_raw` | 入力 |
| `output_topic` | string | `wrench_filtered` | 出力。ゼロ点合わせサービスは`<output_topic>/zero_ftsensor`（`std_srvs/Empty`） |
| `sampling_frequency_hz` | double / Hz | なし（**必須**） | 入力トピックが実際に届く周期。指定しないと起動に失敗する。`500`ではなく`500.0`のようにdoubleで渡すこと |
| `cutoff_frequency_hz` | double / Hz | なし（**必須**） | Butterworthローパスのカットオフ。(0, `sampling_frequency_hz`/2) |
| `filter_order` | int | 3 | Butterworthの次数。[1, 8] |
| `data_window` | int / サンプル | 100 | ゼロ点合わせを許可するまでに受信が必要なサンプル数 |
| `initial_zero` | bool | true | `data_window`サンプル受信後に、自動でゼロ点を合わせる。合わせるまでは出力しない |
| `disable_filtering` | bool | false | フィルタを通さず、オフセットだけを引く |

係数は、ノードの中で`scipy.signal.butter(filter_order, cutoff_frequency_hz / (0.5 * sampling_frequency_hz))`と同じ計算で求めます（scipy 1.15.3と一致することをテストで確認）。
フィルタは、メッセージを1通受け取るたびに1段進みます。そのため`sampling_frequency_hz`には、センサ内部の更新周期ではなく、入力トピックが実際に届く周期を指定してください。
既定値を持たせていないのは、周期が合わないとカットオフが黙ってずれるためです（例：500 Hzのトピックを100 Hzとして設計すると、カットオフは5倍になる）。

次数が高くカットオフが低いと、係数（伝達関数の形）が数値的に不安定になります。例えば次数8・カットオフ5 Hz・入力500 Hzでは、直流ゲインに約3e-5の誤差が出ます（scipyの係数でも約3e-6）。通常は既定の3次で使ってください。

移植元からの変更点:

- **NaN/Infを含むサンプルは捨てます。** フィルタにも通さず、出力もしません。IIRの内部状態にNaNが入ると、その後の出力がすべてNaNになってしまうためです。出力が止まったことは、`WrenchWatchdog`がstaleとして検知します。
- 係数（`filter_b` / `filter_a`）を渡す代わりに、`sampling_frequency_hz`・`cutoff_frequency_hz`（どちらも必須）・`filter_order`からノードの中で計算します。移植元にあった例示用の係数（既定値）は削除しました。
- `initial_zero`がtrueのときは、ゼロ点を合わせる前の値（オフセットが引かれていない値）を出力しません。

## WrenchWatchdog

`cartesian_controllers`の`cartesian_force_controller`などは、入力が途絶えても最後に受け取った値で制御を続けます。NaNを受け取った場合は無視するので、これも直前の値で続けることになります。

このノードは同じトピックを別プロセスで監視します。監視対象のコントローラのどれかがactiveな間に、次のいずれかになると、controller_managerにswitchを要求します。

- 一度も受信していない
- 最後の受信から`stale_timeout_ms`を超えた
- 最後のメッセージにNaN/Infが含まれる

switchの内容:

1. activeな監視対象をdeactivateし、同時に`fallback_controller`をactivateする（STRICT）。joint trajectory controllerは、activateされた時点の位置を保持します。
2. 1が拒否された場合は、監視対象のdeactivateだけを行う（BEST_EFFORT）。

故障が続いていて、監視対象がまだactiveなら、`check_period_ms`ごとにswitchを再試行します。
switchのサービスが使えない、またはdeactivateだけの要求まで拒否された場合は、`escalation_service`（`std_srvs/Trigger`）を故障1回につき1度だけ呼びます。
URでは`/dashboard_client/stop`（プログラム停止）を指定する想定です。
watchdogが力制御のコントローラを自動で再activateすることはありません。

`fallback_controller`には、typeが`joint_trajectory_controller/JointTrajectoryController`または`ur_controllers/ScaledJointTrajectoryController`のコントローラだけを指定できます（完全一致で判定）。
typeは`list_controllers`の結果を受け取るたびに確認し直します（途中で同じ名前の別typeに入れ替えられた場合に備えるため）。
それ以外のtypeだった場合（速度・力のコントローラなど）や、ロードされていない場合は、エラーを出してdeactivateだけを行います。
activateされたときに今の位置を保持すると分かっているのは、joint trajectory controllerだけだからです。
`monitored_controllers`に含まれる名前は`fallback_controller`に指定できません（起動時にエラー）。

switchが成功を返しても、そのあとに送った`list_controllers`の結果で`fallback_controller`がactiveになっていなければ、`escalation_service`を呼びます。

一度でもコントローラの状態を取得できたあとに、`list_controllers`が500 ms以上応答しなくなった場合は、監視対象がactiveかどうか分かりません。この状態で入力の異常があれば、`escalation_service`を呼びます。
起動直後（まだ一度も状態を取得できていない間）は、起動のたびにロボットを止めてしまわないよう、エスカレーションしません。
`switch_controller`の応答が`switch_timeout_ms` + 500 ms以内に来ない場合と、`escalation_service`の応答が500 ms以内に来ない場合は、その要求を破棄してログに残します。

`fallback_controller`が未ロードの場合など、STRICTのswitchが失敗したときは、監視対象をdeactivateするだけになります。
このとき位置指令を出すコントローラは無くなり、その後どう動くかはハードウェア側に依存します。
URドライバでは、指令が来ないと停止します。

| パラメータ | 型 / 単位 | 既定値 | 説明 |
| --- | --- | --- | --- |
| `wrench_topic` | string | `wrench` | 監視するトピック。監視対象のコントローラが実際に読むトピックを指定する |
| `stale_timeout_ms` | double / ms | 50.0 | 入力のタイムアウト |
| `monitored_controllers` | string[] | `[cartesian_force_controller, cartesian_compliance_controller]` | 監視対象のコントローラ |
| `fallback_controller` | string | `scaled_joint_trajectory_controller` | 切り替え先。空にするとdeactivateだけを行う |
| `escalation_service` | string | 空 | switchできないときに呼ぶ`std_srvs/Trigger`サービス。空なら呼ばない |
| `controller_manager` | string | `/controller_manager` | controller_managerの名前 |
| `check_period_ms` | double / ms | 5.0 | 入力を判定する周期 |
| `state_poll_period_ms` | double / ms | 10.0 | コントローラの状態を`list_controllers`で取得する周期 |
| `switch_timeout_ms` | double / ms | 1000.0 | switch要求のタイムアウト |

周期とタイムアウトは、有限で(0, 60000] msの範囲にある必要があります。範囲外ならノードの起動に失敗します。
`WrenchFilter`は、`sampling_frequency_hz`・`cutoff_frequency_hz`が無いか範囲外のとき、`filter_order`が範囲外のとき、`data_window < 1`のとき、または計算した係数の極が単位円の上か外にある（不安定な）ときに、起動に失敗します。

### 限界

- **リアルタイムではありません。** ROSのトピックとサービスを使います。検知から切り替え完了までは、タイムアウトに加えて数ms〜十数msかかります。
- **コントローラのactivate直後は、最大`state_poll_period_ms`だけ監視が始まりません。** activateされたことを知るのが、次の状態取得のときになるためです。
- **watchdog自身が落ちた場合は、何も検知できません。**
- **力やトルクの上限は判定しません。**

## テスト

単体テストは、Butterworthの係数設計（scipyとの一致）、IIRの計算（scipyとの一致、安定性判定）、watchdogの故障判定を対象にしています。

`test_wrench_watchdog_node`は、fakeのcontroller_manager（`list_controllers` / `switch_controller`）とエスカレーション用サービスを相手に、
watchdogのコンポーネントを実際に読み込んで動かす統合テストです（`ROS_DOMAIN_ID=96`で実行）。次の場合を確認しています。

- 入力が正常なら切り替えない／監視対象がinactiveなら入力が無くても切り替えない
- stale・NaNで、監視対象をdeactivateしてfallbackをactivateする（STRICT）。NaNは、正常な受信を確立したあとに500 Hzで送り続け、staleの期限（50 ms）より前に切り替わることを確認
- switchが成功を返したのにfallbackがactiveにならない、またはswitchの直前にfallbackが別typeに入れ替えられた → エスカレーション
- switch前に送った`list_controllers`の応答が、switch後に届く（順序の入れ替わり） → その応答では検証せず、エスカレーションしない。そのあと新しく取得した応答で検証を再開する
- STRICTが拒否される → deactivateだけ（BEST_EFFORT）／それも拒否される → エスカレーション（1回）
- fallbackのtypeが違う、または途中で別typeに入れ替えられた → fallbackはactivateしない
- `fallback_controller`が`monitored_controllers`に含まれる → 起動時にエラー

onolab_ros2のUR7e模擬ハードウェアで手動確認した結果は次のとおりです（2026-10-11）。

- フィルタのcontainerを`kill -9`したとき：52.8 msでstaleを検知し、56.6 msで`cartesian_compliance_controller`から`scaled_joint_trajectory_controller`への切り替えが完了した。手先の位置は変化しなかった
- 模擬ハードウェアの力センサ値（NaN）しか来ない状態で`cartesian_compliance_controller`をactivateしたとき：フィルタはNaNを捨てて出力しないので、watchdogが「未受信」として即座にJTCへ切り替えた

```bash
colcon build --packages-select wrench_filter
colcon test --packages-select wrench_filter && colcon test-result --verbose
```
