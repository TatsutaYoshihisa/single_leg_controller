## single_leg_controllerについて
ここにはver1とver2があります。<br>
CMakeList.txtを書き換えることで切り替えることが出来ます。<br>
ですが、ver1のみを使用してください。<br>
ver2はsingle_leg_contorllerには脚の識別機能を追加して、6つ同時に立ち上げることが可能になっています。<br>
しかし、6つのsingle_leg_controllerを同時に立ち上げると、<br>

```bash
[ERROR] [1745486963.946082548]: Failed to sync read positions: [TxRxResult] Incorrect status packet!
[ERROR] [1745486963.954166152]: Failed to sync read positions: [TxRxResult] Incorrect status packet!
[ERROR] [1745486963.988380729]: Failed to sync read positions: [TxRxResult] Incorrect status packet!
```

というエラーが発生します。<br>
このエラーは1つのUSBポートに2つのsingle_leg_controllerがアクセスしようとして競合しているため発生していると予想しています。<br>
この解決策としてはU2D2を6つにすることは出来ないので**dual_leg_controller**を作成します。<br>
複数の脚を同時に操作したい場合はdual_leg_controllerを探してください。<br><br>
2025/04/24--F24011 : 龍田佳尚