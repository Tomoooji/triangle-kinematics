# triangle-kinematics

2軸ロボットアームの逆運動学・順運動学を計算するArduino互換ライブラリ

## 概要

このライブラリは、2つの回転関節（肩・肘）を持つロボットアームの：
- **順運動学 (Forward Kinematics)**: 関節角度 → 手先位置の計算
- **逆運動学 (Inverse Kinematics)**: 手先位置 → 関節角度の計算

を行います。余弦定理を使用した正確な計算と、到達不可能な位置への安全な検証機構を備えています。

## 構造図

```
 (upper_B : true)       (upper_B : false)
                     |               C＿
                     |             ・/ 。
           B         |           ・ /
         ／*\        |         ・  /
       ／    \       |       ・   /
     ／       \      |     ・    /
   ／          \ 。　 |   ・   * /
  A・・・・・・・・C―――  | A――――――B 
 ------------------------------------------
 A : Shoulder of arm (肩)
 B : Elbow of arm (肘)
 C : Hand/End effector (手先)
 - - - - - - - - - - - - - - -
 ang_B : ∠ABC (肘の内角)
 ang_C : 手先の角度
```

## 使用方法

### 基本的な初期化

```cpp
#include "triangle.h"

// Triangle(上腕長, 下腕長, 肩の角度, 肘の角度, 肘の位置)
Triangle arm(50, 40, 0, PI/2, true);
// 上腕50mm、下腕40mm、肘が上の位置で初期化
```

### 手先位置を指定（直交座標）

```cpp
// 絶対位置指定
if (arm.C_xy(30, 20)) {  // (30, 20)へ移動
  Serial.println("到達可能");
} else {
  Serial.println("可動域外");  // 自動的に前の状態にロールバック
}

// 相対移動
arm.C_xy(5, 0, true);  // 現在位置から相対的に(5, 0)移動
```

### 手先位置を指定（極座標）

```cpp
// 距離r、角度aで指定
arm.C_ra(60, PI/4);      // 距離60mm、角度45°
arm.C_ra(10, 0.1, true); // 相対移動
```

### 関節角度を制御

```cpp
// 肩の角度を設定
float shoulder_angle = arm.A_ang_rad(PI/3);        // ラジアン
float shoulder_angle_deg = arm.A_ang_deg(60);      // 度数法

// 肘の角度を設定
float elbow_angle = arm.B_ang_rad(PI/2);           // ラジアン
float elbow_angle_deg = arm.B_ang_deg(90);         // 度数法

// 手先の角度を取得
float hand_angle = arm.C_ang_rad();                // ラジアン
float hand_angle_deg = arm.C_ang_deg();            // 度数法
```

### 肘の位置を切り替え

```cpp
// 肘を上と下で切り替え
if (arm.B_type(false)) {
  Serial.println("肘の位置が変わりました");
}
```

## API リファレンス

### コンストラクタ

```cpp
Triangle(float lenAB, float lenBC, float angA = 0, float angB = 0, bool is_B_upper = true)
```
- `lenAB`: 上腕の長さ [mm]
- `lenBC`: 下腕の長さ [mm]
- `angA`: 初期の肩の角度 [rad]
- `angB`: 初期の肘の内角 [rad]
- `is_B_upper`: 肘が上（true）か下（false）か

### メソッド一覧

| メソッド | 説明 | 戻り値 |
|---------|------|--------|
| `C_xy(x, y, as_delta)` | 手先を直交座標で指定 | bool (成功/失敗) |
| `C_ra(r, a, as_delta)` | 手先を極座標で指定 | bool (成功/失敗) |
| `A_ang_rad(a, as_delta)` | 肩の角度を設定 [rad] | float (肩の角度) |
| `A_ang_deg(a, as_delta)` | 肩の角度を設定 [deg] | float (肩の角度) |
| `B_ang_rad(a, as_delta)` | 肘の角度を設定 [rad] | float (肘の角度) |
| `B_ang_deg(a, as_delta)` | 肘の角度を設定 [deg] | float (肘の角度) |
| `C_ang_rad()` | 手先の角度を取得 [rad] | float |
| `C_ang_deg()` | 手先の角度を取得 [deg] | float |
| `B_type(upper)` | 肘の位置を切り替え | bool (変更有無) |
| `is_in_range()` | 手先が可動域内か確認 | bool |

### パラメータの説明

#### `as_delta`パラメータ
- `true`: 相対移動（現在位置からの変化量）
- `false`: 絶対位置指定（直接値を設定）

## 数学的背景

### 順運動学 (Forward Kinematics)

肘の内角 `∠ABC` から手先位置を計算します：

```
C.x = cos(±(π - ∠ABC) + ∠BAC) × lenAB + B.x
C.y = sin(±(π - ∠ABC) + ∠BAC) × lenAB + B.y
```

### 逆運動学 (Inverse Kinematics)

**余弦定理**を使用して関節角度を計算します：

```
cos(∠ABC) = (AB² + BC² - AC²) / (2·AB·BC)
cos(∠BAC) = (AB² + AC² - BC²) / (2·AB·AC)
```

到達可能な範囲：`|AB - BC| < AC < AB + BC`

## 特徴

✅ **数学的に正確**: 余弦定理に基づく厳密な計算  
✅ **安全性**: 可動域外の位置は自動的にロールバック  
✅ **軽量**: Arduino互換マイコンで動作  
✅ **柔軟性**: 絶対位置・相対移動・両座標系に対応  

## 制限事項

- ⚠️ このライブラリは2自由度（肩・肘）のみに対応しています
- ⚠️ 手首（3番目の関節）の角度制御は別途実装が必要です
- ⚠️ リンクの衝突検出は実装されていません

## 使用例

```cpp
#include "triangle.h"
#include <Servo.h>

Triangle arm(36, 26, 0, PI/2, true);  // サクアーム用設定
Servo servo_A, servo_B, servo_C;

void setup() {
  servo_A.attach(9);
  servo_B.attach(10);
  servo_C.attach(11);
}

void loop() {
  // 手先を目標位置へ移動
  if (arm.C_xy(20, 15)) {
    servo_A.write(degrees(arm.A_ang_rad()));
    servo_B.write(degrees(arm.B_ang_rad()));
    servo_C.write(degrees(arm.C_ang_rad()));
  }
  delay(100);
}
```

## 更新履歴

### v1.0.1
- ✅ コンストラクタの変数名を修正
- ✅ `A_ang_rad()` の条件分岐を修正
- ✅ ゼロ除算対策を強化 (epsilon値: 0.00001 → 0.001)

### v1.0.0
- 初期リリース

## ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照してください

## 参考資料

- ロボットアームの運動学: [Robot Kinematics](https://en.wikipedia.org/wiki/Forward_kinematics)
- 余弦定理の応用
