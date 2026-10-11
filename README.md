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
| `filter_b`, `filter_a` | double[] | なし（必須） | IIRの係数（`scipy.signal.butter`と同じ形式）。**入力トピックが実際に届く周期で設計すること** |
| `data_window` | int / サンプル | 100 | ゼロ点合わせを許可するまでに受信が必要なサンプル数 |
| `initial_zero` | bool | true | `data_window`サンプル受信後に、自動でゼロ点を合わせる。合わせるまでは出力しない |
| `disable_filtering` | bool | false | フィルタを通さず、オフセットだけを引く |

フィルタは、メッセージを1通受け取るたびに1段進みます。そのため係数は、センサ内部の更新周期ではなく、入力トピックの周期で設計してください。

移植元からの変更点:

- **NaN/Infを含むサンプルは捨てます。** フィルタにも通さず、出力もしません。IIRの内部状態にNaNが入ると、その後の出力がすべてNaNになってしまうためです。出力が止まったことは、`WrenchWatchdog`がstaleとして検知します。
- `filter_b` / `filter_a`は必須です。移植元にあった例示用の係数（既定値）は削除しました。
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

`fallback_controller`には、typeに`JointTrajectoryController`を含むコントローラだけを指定できます（`scaled_joint_trajectory_controller`なども可）。
それ以外のtypeだった場合（速度・力のコントローラなど）は、最初の状態取得の時点でエラーを出し、以後はdeactivateだけを行います。
activateされたときに今の位置を保持すると分かっているのは、joint trajectory controllerだけだからです。

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
`WrenchFilter`は、`data_window < 1`のとき、または`filter_a`の極が単位円の上か外にある（不安定な）ときに、起動に失敗します。

### 限界

- **リアルタイムではありません。** ROSのトピックとサービスを使います。検知から切り替え完了までは、タイムアウトに加えて数ms〜十数msかかります。
- **コントローラのactivate直後は、最大`state_poll_period_ms`だけ監視が始まりません。** activateされたことを知るのが、次の状態取得のときになるためです。
- **watchdog自身が落ちた場合は、何も検知できません。**
- **力やトルクの上限は判定しません。**

## テスト

単体テストは、IIRの計算（scipyとの一致、安定性判定）と、watchdogの故障判定を対象にしています。
ROSノード間の統合テストは含みません。onolab_ros2のUR7e模擬ハードウェアで手動確認した結果は次のとおりです（2026-10-11）。

- フィルタのcontainerを`kill -9`したとき：52.8 msでstaleを検知し、56.6 msで`cartesian_compliance_controller`から`scaled_joint_trajectory_controller`への切り替えが完了した。手先の位置は変化しなかった
- 模擬ハードウェアの力センサ値（NaN）しか来ない状態で`cartesian_compliance_controller`をactivateしたとき：フィルタはNaNを捨てて出力しないので、watchdogが「未受信」として即座にJTCへ切り替えた

```bash
colcon build --packages-select wrench_filter
colcon test --packages-select wrench_filter && colcon test-result --verbose
```
