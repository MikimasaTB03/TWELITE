# TWELITE

App_Tagファームウエアに赤外線センサーMLX90614とパルスカウンターの機能を追加しました。
どちらも動作確認のみで校正など行っていません。それでもよければ使ってください。

Sensor Modeは以下に設定しています。  
0x42 赤外線センサーMLX90614  
0x43 パルスカウンター  

赤外線温度センサーは変換時間を200msとしているので、スリープ周期はそれ以上必要です。
また、省電力のため電源をON/OFFしているので、パスコンをつけることができず、精度もあまり良くないようです。  


パルスカウンターはスリープ時間内にカウントし、スリープ起床時に読み出します。
スリープ周期を１０秒に設定すれば１０秒間のカウント数を返します。
リードスイッチの入力に対応してDebounceを8samplesに設定しています。  


※ソースコードにはモノワイヤレス株式会社様公式ファームウェアのソースからの
  引用が含まれます。取り扱いに際しては著作権表記を確認の上、
 「モノワイヤレスソフトウェア使用許諾契約書」の内容を遵守して下さい。

※ソースコードや図面などには誤りが含まれている可能性があります。
  そのことがいかなる損害に繋がったとしても当方は一切の責任を負いません。あらかじめご了承下さい。


Mikimasa
