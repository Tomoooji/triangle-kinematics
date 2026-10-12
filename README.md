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



## ライセンス

MIT License - 詳細は [LICENSE](LICENSE) を参照してください

## 参考資料

- ロボットアームの運動学: [Robot Kinematics](https://en.wikipedia.org/wiki/Forward_kinematics)
- 余弦定理の応用

---
2026-10-12  
Tomoooji  