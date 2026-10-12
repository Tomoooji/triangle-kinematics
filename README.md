# TwoLink-kinematics

2リンクマニピュレータ(2軸ロボットアーム)の逆運動学・順運動学を計算するArduino互換ライブラリ  

## 概要

このライブラリは、2つの回転関節（肩・肘）を持つロボットアームの：
- **順運動学 (Forward Kinematics)**: 関節角度 → 手先位置の計算
- **逆運動学 (Inverse Kinematics)**: 手先位置 → 関節角度の計算

を行います。余弦定理を使用した正確な計算と、到達不可能な位置への安全な検証機構を備えています。

## 構造図

```
 (isJointUpper : true)       (isJointUpper : false)
                        |               End___
                        |              ・/ +
           Joint        |            ・ /
           ／*\         |           ・ /
         ／    \        |         ・  /
       ／       \       |       ・   /
     ／          \ + 　 |     ・   */
  Base・・・・・・End――― | Base―――Joint 
 - - - - - - - - - - - - - - - - - - - - - - - - - 
 * : Joint Angle
 + : End Slope
```

## 使用方法

### 1. 初期化
本ライブラリに加え、依存ライブラリである mathmatics_tmj が必要。
```C++
#include <Arduino.h>
#include <twolink-kinematics.h>

// リンク長: Base-Joint = 100, Joint-End = 80
// 関節を上側に配置
// 初期角度: Base = 0 rad, Joint = 1.0 rad

TwoLink arm(
    100.0f,
    80.0f,
    true,
    RadianAbsPi<float>(0.0f),
    RadianAbsPi<float>(1.0f)
);
```
TwoLink のコンストラクタには、以下の引数を指定する。  
| **引数** | **説明** |
| --- | --- |
| lengthBaseJoint | Base から Joint までのリンク長 |
| lengthJointEnd | Joint から End までのリンク長 |
| isJointUpper | 関節を上側に配置する場合は true、下側の場合は false |
| initAngleBase | Base 側の初期角度 |
| initAngleJoint | Joint 側の初期角度 |
| initBasePos | Base の初期座標（省略時は (0, 0)） |

角度の単位はラジアンである。initAngleBase と initAngleJoint は省略できる。

### 2. 順運動学

関節角度を指定し、各関節の座標を更新する。
```C++
arm.setAngles(
    RadianAbsPi<float>(0.5f),
    RadianAbsPi<float>(1.0f)
);
```
setAngles() は、指定した角度から各関節の座標を計算する。  
現在の角度からの変化量を指定する場合は、第三引数に true を指定する。
```C++
arm.setAngles(
    RadianAbsPi<float>(0.1f),
    RadianAbsPi<float>(-0.2f),
    true
);
```
### 3. 逆運動学
手先の目標座標を指定し、関節角度と各関節の座標を更新する。
```C++
if (arm.setEndPosition(Vector2D<float>(120.0f, 50.0f))) {
  // 目標座標に到達可能
} else {
  // 目標座標に到達不可能
}
```

setEndPosition() は、目標座標に到達可能な場合のみ状態を更新し、成功時は true、失敗時は false を返す。  
現在の End の座標からの相対移動量を指定する場合は、第二引数に true を指定する。  
```C++
arm.setEndPosition(Vector2D<float>(10.0f, 5.0f), true);
```

### 4. 角度・座標の取得
各関節の座標や角度は、以下のメンバ関数から取得できる。
```C++
// 各関節の座標
Vector2D<float> base  = arm.getBase();
Vector2D<float> joint = arm.getJoint();
Vector2D<float> end   = arm.getEnd();

// 各関節の角度
RadianAbsPi<float> baseAngle  = arm.getBaseAngle();
RadianAbsPi<float> jointAngle = arm.getJointAngle();

// 手先側のリンクの傾き
RadianAbsPi<float> endSlope = arm.getEndSlope();
```
Vector2D<float> の座標は getX() と getY()、角度は float 型への変換によってラジアン値を取得できる。
```C++
float x = arm.getEnd().getX();
float y = arm.getEnd().getY();
float angle = arm.getBaseAngle();
```
角度を個別に変更する場合は setBaseAngle() と setJointAngle() を使用できる。いずれも第二引数に true を指定すると、現在の角度に対する変化量として扱う。

## 注意事項
- リンク長には正の値を指定すること。
- 逆運動学では、リンク長と目標座標によって到達できる範囲が決まる。
- setEndPosition() が false を返した場合、関節の状態は更新されない。
- 角度は度数法ではなくラジアンで指定する。

## ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照してください

## 参考資料

- ロボットアームの運動学: [Robot Kinematics](https://en.wikipedia.org/wiki/Forward_kinematics)
- 余弦定理の応用

---
2026-10-12  
Tomoooji  