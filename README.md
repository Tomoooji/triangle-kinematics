# triangle-kinematics
general class for calculating angles and positions of 2 axis robot arm


```
 (lager_B : true)      (lager_B : false)
                     |               C＿
                     |             ・/ 。
           B         |           ・ /
         ／*\        |         ・  /
       ／    \       |       ・   /
     ／       \      |     ・    /
   ／          \ 。　 |   ・   * /
  A・・・・・・・・C―――  | A――――――B 
 ------------------------------------------
 A : Shoulder of arm
 B : ellbow of arm
 C : hand of arm
 - - - - - - - - - - - - - - -
 ang_B : * (= ∠ABC)
 ang_C : 。
 - - - - - - - - - - - - - - -
          AB^2 + BC^2 - AC^2
 cosABC = ―――――――――――――――――――
              2 * AB * BC
```
