RoKiチュートリアル: ロボットアームの逆動力学
====================================================================================================
Copyright (C) Tomomichi Sugihara (Zhidao)

 - 2024.08.03. 作成 Zhidao
 - 2024.08.06. 最終更新 Zhidao

----------------------------------------------------------------------------------------------------

# ロボットの質量特性

[前回](tutorial_roki003.md)PUMAにさせた動作はそれなりに激しいものでしたので、実際にロボットにさせようとしたとき、関節にはどれくらいの負荷がかかるのかは当然気になります。
このような、ある運動を実現するために関節が発揮しなければならないトルクを逆算する計算を、**逆動力学**と呼びます。

力学的な因果関係から言えば、関節に搭載されたモータの端子間に電圧をかけると、回路インピーダンスと逆起電力に応じて巻線を流れる電流が決まり、その電流が受けるローレンツ力が駆動トルクとなり、それが身体の慣性に反比例して関節角加速度を産み出し、積分されて角速度、さらには角度と変化していく…というのが、運動が生じる流れです。
この流れに沿った計算が順動力学、この流れを逆にたどる計算が逆動力学である、と考えれば理解が容易になると思います。
ただし、モータや回路のダイナミクスまで考慮することは稀で、通常は運動（主に加速度）と関節トルクの関係すなわち運動方程式に基づく計算に留まります。

さて、運動と力とを結びつけるためには、慣性の情報、具体的にはロボットを構成する各リンクの質量（慣性0次モーメント）・重心位置（慣性1次モーメント）・慣性テンソル（慣性2次モーメント）が必要です。
これらは全て、リンク座標系におけるリンクの質量分布から決まるもので、まとめて**質量特性**と呼ばれます。

`puma.zkc`の`[link]`フィールドを見てみましょう。
```
[link]
name : base
jointtype : fixed
shape: base
shape: post

[link]
name : link1
jointtype : revolute
max : 180
min :-180
DH: { 0, 0, 0.3, 0 }
inertia: {
 0, 0, 0,
 0, 0, 0,
 0, 0, 0.35 }
parent : base
shape: shoulderroot

[link]
name : link2
jointtype : revolute
max : 180
min :-180
DH: { 0, -90, 0.05, 0 }
mass: 17.4
inertia: {
 0.13, 0, 0,
 0, 0.524, 0,
 0, 0, 0.539 }
COM: { 0.068, 0.006, -0.016 }
parent : link1
shape: shoulder
shape: upperarm

...
```
最初の`base`リンクは固定された土台なので良いとして、`link1`ではzz成分のみ0.35でそれ以外0の`inertia`が与えられています。`link2`ではさらに、`mass`、対角成分が全て非零な`inertia`、そして`COM`が与えられていることが分かります。

※それ以外に`DH`というプロパティも目につくかと思います。これは **（拡張）DHパラメータ** と呼ばれるもので、別の回で説明します。

`link1`の`inertia`はなぜzz成分しか与えられていないのかについて、別に他の成分も厳密に与えられていても構わないのですが、`link1`の（回転）関節は水平面内を旋回するだけなので、これ以外の質量特性は運動方程式に現れません。
このため、計算に関係ない成分は全て0としている、とういうわけです。
一方、`link2`は3次元的に動くので、全ての成分が計算に関係してきます。
対称性の高い部品で形状を近似しているため、慣性テンソルが対角行列となっています。
ちなみにこれらの値は、1986年に米スタンフォード大のOussama Khatibらのグループが頑張って同定したものです。

Brian Armstrong, Oussama Khatib and Joel Burdick, "The Explicit Dynamic Model and Inertial Parameters of the PUMA 560 Arm," in Proceedings of the 1986 IEEE International Conference on Robotics and Automation, pp.510-518, 1986.

なお、`inertia`は必ず正定値対称行列でなければなりません。


# 逆動力学

質量特性が与えられれば、運動（変位・速度・加速度）と力（トルク）とを結び付けることができます。
次のようなコードを作ってみましょう。

```C
#include <roki/rk_chain.h>

#define T 3.0

#define STEP 100

void set_joint_angle(zVec q, zVec dq, zVec ddq, double t)
{
  double phase, omega;

  omega = zPIx2 / T;
  phase = omega * t;
  zVecSetElem( q, 0, zDeg2Rad(45) * sin(  phase) );
  zVecSetElem( q, 1, zDeg2Rad(30) * sin(2*phase) );
  zVecSetElem( q, 2, zDeg2Rad(60) * sin(2*phase) - zPI_2 );
  zVecSetElem( dq, 0,   omega * zDeg2Rad(45) * cos(  phase) );
  zVecSetElem( dq, 1, 2*omega * zDeg2Rad(30) * cos(2*phase) );
  zVecSetElem( dq, 2, 2*omega * zDeg2Rad(60) * cos(2*phase) );
  zVecSetElem( ddq, 0,-  zSqr(omega) * zDeg2Rad(45) * sin(  phase) );
  zVecSetElem( ddq, 1,-4*zSqr(omega) * zDeg2Rad(30) * sin(2*phase) );
  zVecSetElem( ddq, 2,-4*zSqr(omega) * zDeg2Rad(60) * sin(2*phase) );
}

int main(int argc, char *argv[])
{
  rkChain robot;
  zVec q, dq, ddq, trq;
  int i;
  double t;

  if( !rkChainReadZTK( &robot, "puma" ) ||
      !( q = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( dq = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( ddq = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( trq = zVecAlloc( rkChainJointSize(&robot) ) ) )
    return EXIT_FAILURE;

  for( i=0; i<STEP; i++ ){
    set_joint_angle( q, dq, ddq, ( t = T*(double)i/STEP ) );
    rkChainFK( &robot, q );
    rkChainID( &robot, dq, ddq );
    rkChainGetJointTrqAll( &robot, trq );
    printf( "%g %g %g %g %g %g %g %g %g %g\n", t, zVecElem(q,0), zVecElem(q,1), zVecElem(q,2), zVecElem(dq,0), zVecElem(dq,1), zVecElem(dq,2), zVecElem(trq,0), zVecElem(trq,1), zVecElem(trq,2) );
  }
  zVecFreeAtOnce( 4, q, dq, ddq, trq );
  rkChainDestroy( &robot );
  return EXIT_SUCCESS;
}
```

`set_joint_angle()`関数と`main()`関数の前半は、[前回](tutorial_roki003.md)示したものと同じです。
`for`ループの中で、
まず関節変位ベクトル`q`を作成し、`rkChainFK()`に与えて順運動学を解きます。
続いて呼んでいる`rkChainID()`で、逆動力学を解いています。
これは、次のように書いても同じ結果が得られます。
```C
    rkChainSetJointRateAll( &robot, dq, ddq );
    rkChainUpdateID( &robot );
```
求まった全関節のトルクの値は、`rkChainGetJointTrqAll()`関数で取り出すことが出来ます。
`trq`は**関節トルクベクトル**とか**一般化座標に対応する一般化力ベクトル**などと呼ばれます。

特定のリンク、例えばリンク1の関節トルクだけを取り出したい場合は、次のように出来ます。
```C
  double torque;
  rkChainLinkJointGetTrq( &chain, 1, &torque );
```

上記プログラムをコンパイル・実行すると、`link1`、`link2`、`link3`の関節角度、同3リンクの関節速度、同3リンクの関節トルクがこの順番で標準出力に出力されます。
これをグラフにプロットすると次のようになります。

<img width=1024 alt="PUMA逆動力学テスト動作の関節角度" src="fig/puma_inversedynamics_q.png">
<img width=1024 alt="PUMA逆動力学テスト動作の関節トルク" src="fig/puma_inversedynamics_tau.png">

上が関節角度、下が関節トルクです（横軸の単位はsです）。
大雑把な見方ですが、

 - 関節角度は正弦波状に変化するので、関節加速度はそれと逆位相になり、関節トルクもおおよそそれに比例する（はず）
 - `link1`関節（旋回軸）が1往復する間に`link2`と`link3`の関節は2往復し、それと連動して旋回軸から見た重心までの距離も近づいて離れるを2回繰り返す。したがって`link1`関節トルクは1/3Hzの波と2/3Hzの波を合成した波形になる

くらいの推定は出来、傾向がこれと整合する結果が得られていることも分かります。

また、速度-トルク曲線は次のようになります（絶対値をとっています）。

<img width=640 alt="PUMA逆動力学テスト動作の関節速度-関節トルク曲線" src="fig/puma_inversedynamics_dq-tau.png">

これをモータカタログにある速度-トルク曲線と比較すれば、どのくらい余裕があるかが分かるでしょう。


# 逆動力学演算の中身を少しだけ

上で紹介した`rkChainID()`と`rkChainUpdateID()`は、実は関数ではなくマクロです。
関数プロトタイプは`rk_chain.h`の中で次のように宣言されています。
```C
void rkChainUpdateID_G(rkChain *chain, zVec6D *g);
#define rkChainUpdateID(chain)     rkChainUpdateID_G( chain, RK_GRAVITY6D )
#define rkChainUpdateID0G(chain)   rkChainUpdateID_G( chain, ZVEC6DZERO )

void rkChainID_G(rkChain *chain, zVec vel, zVec acc, zVec6D *g);
#define rkChainID(chain,vel,acc)   rkChainID_G( chain, vel, acc, RK_GRAVITY6D )
#define rkChainID0G(chain,vel,acc) rkChainID_G( chain, vel, acc, ZVEC6DZERO )
```
つまり実体は`rkChainUpdateID_G`、`rkChainID_G()`という関数です。
これらがとる引数`g`は「場の加速度」を意味します。
`rkChainUpdateID()`と`rkChainID()`は`RK_GRAVITY6D`を指定しているので重力場における逆動力学、`rkChainUpdateID0G()`と`rkChainID0G()`は`ZVEC6DZERO`を指定しているので無重力場における逆動力学を、それぞれ計算しているというわけです。

`rkChainUpdateID_G()`は次のように定義されています。
```
void rkChainUpdateID_G(rkChain *chain, zVec6D *g)
{
  rkChainUpdateRateG( chain, g );
  rkChainUpdateWrench( chain );
  rkChainUpdateCOMVel( chain );
  rkChainUpdateCOMAcc( chain );
}
```
`rkChainUpdateRateG()`は、[前回](tutorial_roki003.md)紹介した`rkChainUpdateRate()`と`rkChainUpdateRate0G()`の実体で、場の加速度`g`を含む全リンクの速度・加速度を計算します。
`rkChainUpdateWrench()`は、全リンクの速度・加速度が更新されている前提で、全リンクの関節にかかるレンチ（力・トルクの組）を更新する関数です。
その中で、関節トルクもレンチから求めています。
`rkChainUpdateCOMVel()`と`rkChainUpdateCOMAcc()`はおまけで、リンク全体の重心速度・加速度を更新します。

なお、[前回](tutorial_roki003.md)紹介した`rkChainFKCNT()`も、内部的には逆動力学を解いています。
次のプログラムをコンパイル・実行してみましょう。
```C
#include <roki/rk_chain.h>

#define T 3.0

#define STEP 100

void set_joint_angle(zVec q, double t)
{
  double phase;

  phase = zPIx2 * t / T;
  zVecSetElem( q, 0, zDeg2Rad(45) * sin(  phase) );
  zVecSetElem( q, 1, zDeg2Rad(30) * sin(2*phase) );
  zVecSetElem( q, 2, zDeg2Rad(60) * sin(2*phase) - zPI_2 );
}

int main(int argc, char *argv[])
{
  rkChain robot;
  zVec q, trq;
  double t, dt;
  int i;

  if( !rkChainReadZTK( &robot, "puma" ) ||
      !( q = zVecAlloc( rkChainJointSize(&robot) ) ) ||
      !( trq = zVecAlloc( rkChainJointSize(&robot) ) ) )
    return EXIT_FAILURE;

  dt = T / STEP;
  for( i=0; i<STEP; i++ ){
    set_joint_angle( q, ( t = T*(double)i/STEP ) );
    rkChainFKCNT( &robot, q, dt );
    rkChainGetJointTrqAll( &robot, trq );
    printf( "%g %g %g %g %g %g %g\n", t, zVecElem(q,0), zVecElem(q,1), zVecElem(q,2), zVecElem(trq,0), zVecElem(trq,1), zVecElem(trq,2) );
  }
  zVecFreeAtOnce( 2, q, trq );
  rkChainDestroy( &robot );
  return EXIT_SUCCESS;
}
```
この結果の関節トルクのグラフは次のようになります。

<img width=1024 alt="PUMA逆動力学テスト動作の関節トルク（FKCNT）" src="fig/puma_fkcnt_tau.png">

動作開始時（t=0付近）の値は信頼性が低いですが、おおむね先程の図と似た結果が得られています。
