RoKiチュートリアル: ロボットモデルを作ろう
====================================================================================================
Copyright (C) Tomomichi Sugihara (Zhidao)

 - 2023.01.17. 作成 Zhidao
 - 2023.01.27. 最終更新 Zhidao

----------------------------------------------------------------------------------------------------

# スーパーロボット君登場

RoKiで扱えるロボットモデル作りを体験しましょう。
手始めに、下のようなかっこいいロボットのモデルを作ってみることにします。

<img width=600 alt="スーパーロボット君" src="fig/robot_img.svg">

一番右の図が関節配置を表しています。
シンバルを二つ向かい合わせにしたような黄色い記号は、回転する関節を表しています。
このロボットは、首が鉛直軸まわりに旋回し、両肩と両股関節が前後に振り子のように回転する、計5個の関節を持っていることになります。

# ロボットモデルの寸法と座標系

おおよその寸法を下図のように決めましょう。

<img width=500 alt="ロボット寸法" src="fig/robot_dim.svg">

次に、関節配置に合わせて、リンク座標系を設定します。

 - それぞれの座標系は$z$軸が関節軸と一致していること
 - その際、$z$軸は関節変位の正方向を向いていること（回転関節ならば右ねじの法則から決まる）
 - 関節のつながりと同じように座標系同士がつながっていること

というルールを守っていれば、あとは自由です。
ついでにリンクに名前をつけましょう。
例えば下の右図のように設定できます。

<img width=600 alt="ロボットの関節配置と座標系" src="fig/robot_kinematics.svg">

凡例のように、赤が$x$軸、緑が$y$軸、青が$z$軸をそれぞれ意味しています。
首関節は左回りを正、両肩と両股関節は前に振り上げる方向を正としています。

# 骨組みを作る

骨組みだけならここまでの情報だけで作れます。
次のファイル `super_robot.ztk` を作りましょう。

```
[robot]
name: super_robot

[link]
name: body
jointtype: float

[link]
name: head
parent: body
jointtype: revolute
pos: ( 0, 0, 0.15 )

[link]
name: left_arm
parent: body
jointtype: revolute
pos: ( 0, 0.15, 0 )
att:
 1, 0, 0,
 0, 0,-1,
 0, 1, 0,

[link]
name: right_arm
parent: body
jointtype: revolute
pos: ( 0, -0.15, 0 )
att:
 1, 0, 0,
 0, 0,-1,
 0, 1, 0,

[link]
name: left_leg
parent: body
jointtype: revolute
pos: ( 0, 0.05, -0.2 )
att:
 1, 0, 0,
 0, 0,-1,
 0, 1, 0,

[link]
name: right_leg
parent: body
jointtype: revolute
pos: ( 0, -0.05, -0.2 )
att:
 1, 0, 0,
 0, 0,-1,
 0, 1, 0,
```

これをrk_penで描画してみます。
```
% rk_pen super_robot.ztk -bone
```

-boneオプションを忘れずつけて下さい。

<img width=500 alt="ロボットの骨組み" src="fig/super_robot_bone.png">

太めの円筒が関節を表しています。
手先・足先が無いので骨が関節のところでとまってしまっていて、まだロボットっぽく見えませんが、
リンクが正しく接続されていることと、関節の位置関係・接続関係は分かると思います。

少し解説します。
最初の[robot]タグは、ロボット全体に属するプロパティを書き入れるフィールドを作ります。
今のところプロパティは、ロボットの名前を表すnameだけです。

[link]タグは、個々のリンクに属するプロパティを書き入れるフィールドを作ります。
nameはリンクの名前です。
jointtypeは関節の種類で、後ほど詳しく説明します。
今はbodyリンクだけfloat、他はrevoluteとなっています。
parentには、リンクがつながっている根本に近い側のリンク（親リンク）の名前を書きます。
この例ではbodyリンクがそれ以外の全てのリンク(head, left\_arm, right\_arm, left\_leg, right\_leg)の親になっています。
pos、attはそれぞれ親リンク座標系からの並進、姿勢変換を表します。

headリンク座標系はbodyリンク座標系に対して$z$軸正方向に0.15ずれただけですから、
```
pos: ( 0, 0, 0.15 )
```
としています。
left_armリンク座標系は、位置はbodyリンク座標系に対して$y$軸方向に0.15ずれていますので
```
pos: ( 0, 0.15, 0 )
```
です。
姿勢については次のように考えます。
left_armリンク座標系の$x$軸、$y$軸、$z$軸はそれぞれbodyリンク座標系の$x$軸、$z$軸、$-y$方向を向いています。
単位ベクトルで表せば、それぞれ$(1,0,0)$、$(0,0,1)$、$(0,-1,0)$で、これらを縦に並べたものが姿勢変換行列です。
他のリンクについても同様です。

# 形を作る

最後にロボットの形を作っていきましょう。
色は[optic]タグ、形は[shape]タグでそれぞれフィールドを作り、必要な情報を書き入れます。
色は次の4色に関する情報を追記しましょう。
どこでも構いませんが、特別な事情がなければ[robot]タグフィールドの次が良いでしょう。

```
[optic]
name: cyan
ambient: 0.5, 0.5, 0.5
diffuse: #64c8ff
specular: 0, 0, 0

[optic]
name: yellow
ambient: 0.5, 0.5, 0.5
diffuse: #ffff96
specular: 0, 0, 0

[optic]
name: blue
ambient: 0.5, 0.5, 0.5
diffuse: #6400ff
specular: 0, 0, 0

[optic]
name: pink
ambient: 0.5, 0.5, 0.5
diffuse: #ff96ff
specular: 0, 0, 0
```

nameは適当で構いません。
ambientは環境光（当てるライト）の色、
diffuseは物体の拡散光（地色）の色、
specularは物体の反射光（照り返しの色）の色です。
指定はいずれも赤・緑・青成分の順で、0〜1の値3つ、あるいは頭に#をつけて16進数の値とします（#xxyyzzならばxxが赤、yyが緑、zzが青の成分となります）。
[optic]のプロパティには他にshininess（発光色）、esr（Phongの反射指数）、alpha（透過率）がありますが、
通常は未指定で構いません。

形はプリミティブを使えば比較的簡単に作れます。
次を[optic]タグフィールドの次に追記しましょう。

```
[shape]
name: torso
type: box
optic: cyan
center: ( 0, 0, -0.05 )
width: 0.2
height: 0.2
depth: 0.15

[shape]
name: chest_plate
type: box
optic: yellow
center: ( 0.075, 0, 0 )
width: 0.15
height: 0.1
depth: 0.05

[shape]
name: crotch
type: box
optic: yellow
center: ( 0, 0, -0.15 )
width: 0.05
height: 0.15
depth: 0.1

[shape]
name: neck
type: cylinder
optic: cyan
center: ( 0, 0, 0.05 )
center: ( 0, 0, 0.1 )
radius: 0.025

[shape]
name: head
type: box
optic: cyan
center: ( 0, 0, 0 )
width: 0.2
height: 0.15
depth: 0.2

[shape]
name: left_eye
type: cylinder
optic: yellow
center: ( 0.125, 0.05, 0 )
center: ( 0.1, 0.05, 0 )
radius: 0.025

[shape]
name: right_eye
type: cylinder
optic: yellow
center: ( 0.125, -0.05, 0 )
center: ( 0.1, -0.05, 0 )
radius: 0.05

[shape]
name: left_ear
type: cone
optic: pink
center: ( 0, 0.1, 0 )
vert: ( 0, 0.125, 0 )
radius: 0.05

[shape]
name: right_ear
type: cone
optic: pink
center: ( 0, -0.1, 0 )
vert: ( 0, -0.125, 0 )
radius: 0.05

[shape]
name: arm
type: box
optic: cyan
center: ( 0, -0.05, 0 )
width: 0.2
height: 0.05
depth: 0.05

[shape]
name: hand
type: sphere
optic: blue
center: ( 0, -0.175, 0 )
radius: 0.025

[shape]
name: leg
type: box
optic: cyan
center: ( 0, -0.075, 0 )
width: 0.2
height: 0.05
depth: 0.075

[shape]
name: foot
type: box
optic: blue
center: ( 0.025, -0.175, 0 )
width: 0.05
height: 0.05
depth: 0.15
```

nameは名前、typeは形状の種類、opticは使用する色の名前です。
typeにはbox（直方体）、sphere（球）、cylinder（円柱）、cone（円錐）、ellipsoid（回転楕円体）、ellipticcylinder（楕円柱）、polyhedron（多面体）、nurbs（NURBS曲面）があります。
他のプロパティは、形の種類ごとに異なります。
ここで使用しているbox、sphere、cylinder、coneに関して記せば、

box
 - center : 直方体の中心座標
 - width : $y$軸に沿う辺の長さ
 - height : $z$軸に沿う辺の長さ
 - depth : $x$軸に沿う辺の長さ

sphere
 - center : 球の中心座標
 - radius : 半径

cylinder
 - center : 底面の中心座標（二つ指定する必要があります）
 - radius : 底面の半径

cone
 - center : 底面の中心座標
 - vert : 頂点座標
 - radius : 底面の半径

です。

これらのshapeの情報を各[link]タグフィールドに追記すれば完了です。
```
[link]
name: body
jointtype: float
shape: torso, chest_plate, crotch, neck

[link]
name: head
(略)
shape: head, left_eye, right_eye, left_ear, right_ear

[link]
name: left_arm
(略)
shape: arm, hand

[link]
name: right_arm
(略)
shape: arm, hand

[link]
name: left_leg
(略)
shape: leg, foot

[link]
name: right_leg
(略)
shape: leg, foot
```

-boneオプションを外して、rk_penで見てみましょう。

<img width=500 alt="ロボット" src="fig/super_robot.png">

冒頭に描いたかっこいいスーパーロボット君が現れました。
rk\_penの描画画面はマウスで並進・回転・ズームできます。

<img width=500 alt="ロボット" src="fig/super_robot_iso.png">

ポーズも作ってみましょう。
ターミナルで5番のコマンド "set joint displacement" を選んで、
リンク1（head）の関節角度を45°、
リンク3（right_arm）の関節角度を90°、
リンク4（left_leg）の関節角度を-90°にそれぞれしてみます。

<img width=500 alt="ロボット" src="fig/super_robot_pose.png">

できましたか？
