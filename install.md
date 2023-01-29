mi-libチュートリアル: インストール
====================================================================================================
Copyright (C) Tomomichi Sugihara (Zhidao)

 - 2023.01.17. 作成 Zhidao
 - 2023.01.27. 最終更新 Zhidao
 
----------------------------------------------------------------------------------------------------

# スクリプトによる一括インストール

Ubuntu上で手っ取り早く試用してみたいという方は、
[mi-lib-starter](https://github.com/mi-lib/mi-lib-starter)を使って頂くのが良いです。

```sh
% wget https://github.com/mi-lib/mi-lib-starter/archive/refs/heads/main.zip
% unzip main.zip
% cd mi-lib-starter-main
% ./scripts/mi-lib-install deb
```

これにより、

 1. コンパイルに必要なツール群をaptでインストール
 1. ヘッダファイル、ライブラリ、ユーティリティツールをまとめてdebianパッケージ化
 1. dpkgを使って上記debianパッケージ群をインストール

というプロセスが自動で実行されます。
ただし、途中でsudoを使用しますので、スーパーユーザ権限を持てることが必要条件になります。

この場合、ヘッダファイルは/usr/include/以下に、ライブラリは/usr/lib/以下に、ユーティリティツール群は
/usr/bin/以下にそれぞれインストールされます。インストール先ディレクトリを自分で指定したいという方は、
configファイル内の`PREFIX`を修正（例えば`PREFIX=$HOME/usr`）した上で
```sh
% ./scripts/mi-lib-install
```
として下さい。

また、全てのライブラリをGitHubリポジトリからcloneしたい場合は
```sh
% ./scripts/mi-lib-install clone
```
として下さい。

# スクリプトによる一括アンインストール

debianパッケージとしてインストールした場合には
```sh
% ./scripts/mi-lib-uninstall deb
```
そうでない場合は、
```sh
% ./scripts/mi-lib-uninstall
```
として下さい。

# 中身をもう少し知りたい方に

## コンパイルに必要なツール

上記インストールスクリプトではwget、unzip、rename、fakerootを使っています。
手動で個別インストールされる場合にはこれらは必要ありません。

コンパイルにはmakeが必要です。
コンパイラは選べます。今のところgccとclangはコンパイル可能であることを確認しています。
また、gitがあると幸せになれます。

## サードパーティ製ライブラリ

mi-libは幾つかのサードパーティ製ライブラリを使用します。
ただしこれらの中には必須でないものもあります。また、プロプライエタリ製品は含みません。

 - ZEDA
   - libm
   - \*libxml2
 - RoKi-GL
   - OpenGL (GL, GLU, \*GLX, \*glut)
 - RoKi-FD
 - ZX11
   - x.org (X11, Xpm, Xext)
   - \*libjpeg
   - \*libpng, libz
   - \*libtiff
 - LIW
   - librt, libdl, libpthread

libm、librt, libdl, libpthreadは、gccをご使用ならば既にインストールされているはずです。
左にマークが付いているライブラリは必須ではありません。
それぞれ利用するライブラリの中にあるconfigファイルで使用/不使用を設定できます。
ただし、ZX11でlibpngを使う場合にはlibzは必須になります。

## ライブラリの個別コンパイル＆インストール

それぞれのライブラリのディレクトリにあるconfig.orgファイルをconfigという名前でコピーし、
その中で定義している`PREFIX`を編集することで、インストール先を指定できます。デフォルト
では\$HOME/usrになっています。インストール先ディレクトリにはinclude、lib、binという
サブディレクトリを予め作成しておいて下さい。

ライブラリによっては、他に`CONFIG_USE_*`オプションもあります。これらは必要に応じてyから
nに変えて下さい。

編集が必要なければ、この後の手順で自動的にconfig.orgがconfigという名前でコピーされます。

```sh
% make && make install
```
でコンパイル＆インストールが完了します。

```sh
% make deb
```
とすればdebianパッケージが作成されます。dpkgでインストールして下さい。
この場合は`PREFIX`は無視され、/usr/がインストール先になります。

## 環境変数の設定

環境変数`PATH`に実行ファイルパスを、`LD_LIBRARY_PATH`にライブラリパスをそれぞれ追加します。

例えば`PREFIX=$HOME/usr`ならば、`PATH`には`$HOME/usr/bin`を、`LD_LIBRARY_PATH`には
`$HOME/usr/lib`を追加します。
これはBourne shell family (bash, zsh等)ならば

```sh
% export PATH=$PATH:$HOME/usr/bin
% export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/usr/lib
```

で、あるいはC shell family (csh, tcsh等)ならば

```sh
% set path = ( $path $HOME/usr/bin )
% setenv LD_LIBRARY_PATH $LD_LIBRARY_PATH:$HOME/usr/lib
```

でできます。

以上でmi-libを使う準備は完了です。
