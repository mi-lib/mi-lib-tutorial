mi-libチュートリアル: mi-libをインストールしよう
====================================================================================================
Copyright (C) Tomomichi Sugihara (Zhidao)

 - 2023.01.17. 作成 Zhidao
 - 2025.05.14. 最終更新 Zhidao
 
----------------------------------------------------------------------------------------------------

# スクリプトによるインストール

mi-libに含まれるライブラリを個別にビルド＆インストールすることは可能ですが、Ubuntu上で手っ取り早く使えるようにしてみたいという方には、[mi-lib-starter](https://github.com/mi-lib/mi-lib-starter)を使うことをお勧めします。

まずは、
```sh
% wget https://github.com/mi-lib/mi-lib-starter/archive/refs/heads/main.zip
% unzip main.zip
% cd mi-lib-starter-main
```
として下さい。続けて
```sh
% ls scripts
```
とすれば、次のシェルスクリプトがあるのが分かると思います。

 - install\_dependencies ビルドに必要なツール＆サードパーティライブラリを先にインストールする
 - preprocess ライブラリ・ヘッダ・ツール群のインストール先ディレクトリを自動作成する
 - download ライブラリソースをGitHubリポジトリからダウンロードする
 - build ライブラリをビルド・インストールする
 - install 上記4つのスクリプトを一括で実行する
 - uninstall ライブラリ・ヘッダ・ツール郡をアンインストールする

いちいち考えてインストールするのが面倒だという人は、
```sh
% ./scripts/install deb
```
としてみて下さい。これにより、

 1. コンパイルに必要なツール群をaptでインストール
 1. ヘッダファイル、ライブラリ、ユーティリティツールをまとめてdebianパッケージ化
 1. dpkgを使って上記debianパッケージ群をインストール

というプロセスが自動で実行されます。
ただし、途中でsudoを使用しますので、スーパーユーザ権限を持てることが必要条件になります。

この場合、ヘッダファイルは/usr/include/以下に、ライブラリは/usr/lib/以下に、ユーティリティツール群は/usr/bin/以下に、それぞれインストールされます。
インストール先ディレクトリを自分で指定したいという方は、configファイル内の`PREFIX`を修正（例えば`PREFIX=$HOME/usr`）した上で
```sh
% ./scripts/install
```
として下さい。
この場合は、ライブラリ本体のパスを環境変数 `LD_LIBRARY_PATH`に、ユーティリティツールのパスを環境変数 `PATH`に、それぞれ追加する必要があります。
`PREFIX=$HOME/usr`ならば、`PATH`には`$HOME/usr/bin`を、`LD_LIBRARY_PATH`には`$HOME/usr/lib`を追加するということです。
これは手動で行わなければなりません。
方法が、ユーザの使用しているシェルの種類に依存するからです。

Bourne shell family (bash, zsh等)ならば

```sh
% export PATH=$PATH:$HOME/usr/bin
% export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/usr/lib
```

あるいはC shell family (csh, tcsh等)ならば

```sh
% set path = ( $path $HOME/usr/bin )
% setenv LD_LIBRARY_PATH $LD_LIBRARY_PATH:$HOME/usr/lib
```

として下さい。

以上でmi-libを使う準備は完了です。



---
# スクリプトによる一括アンインストール

debianパッケージとしてインストールした場合には
```sh
% ./scripts/uninstall deb
```
そうでない場合は、
```sh
% ./scripts/uninstall
```
として下さい。


---
# 中身をもう少し知りたい方のために

## スクリプトinstall\_dependencies

前述のinstallスクリプトでは、最初にinstall\_dependenciesを実行します。
この中ではまず、次のツール群をaptでインストールします。

 - wget
 - unzip
 - rename
 - make
 - pkg-config
 - fakeroot
 - git

これらのうち、mi-libを使ったプログラム開発に必要なのは make と pkg-config のみです。
wget、unzip、renameはdownloadスクリプトの中だけで使うもので、ダウンロードを手動でするならば不要です。
fakerootはdebパッケージ作成に必要です。
gitは必須ではありませんが、あれば幸せになれます。

コンパイラはインストール済であることを仮定しています。
デフォルトではgccを使用しますが、他のものに変えることも可能です。
今のところ、clangでもコンパイル可能であることを確認しています。

次に、幾つかのサードパーティ製ライブラリをaptでインストールします。
mi-libで使われるライブラリは次の通りです。プロプライエタリ製品は含まれていません。

 - ZEDA
   - libm
   - \*libxml2
 - Zeo
   - \*liblzf
 - RoKi-GL
   - OpenGL (GL, GLU, \*GLX, \*GLUT, \*GLFW, \*GLEW)
 - RoKi-FD
 - ZX11
   - x.org (X11, Xpm, Xext)
   - \*Xfg (Xft, fontconfig, freetype)
   - \*ImageMagick (MagickCore, MagickWand)
   - \*libjpeg
   - \*libpng, libz
   - \*libtiff
 - LIW
   - librt, libdl, libpthread

左に\*マークが付いているライブラリは必須ではありません。
それぞれ利用するライブラリの中にあるconfigファイルで使用/不使用を設定できます。
ただし、ZX11でlibpngを使う場合にはlibzは必須になります。

libm、librt, libdl, libpthreadは、gccをご使用ならば既にインストールされているはずなので、aptでのインストール対象から除かれています。


## スクリプトpreprocess

installスクリプトで二番目に実行されるのが、preprocessです。

mi-libを使ってプログラム開発するためには、通常のライブラリと同様に、ヘッダファイル、ライブラリ本体、ユーティルティツールを参照可能な場所に置く必要があります。
これらは、configファイル内で定義された`PREFIX`に対しそれぞれ`${PREFIX}`/include、`${PREFIX}`/lib、`${PREFIX}`/bin に置かれると仮定されます。
`PREFIX`が空（デフォルトではそのようになっています）の場合は、`${HOME}/usr` で置き換えられます。
他の場所にしたい場合はconfigファイルを編集して下さい。

スクリプトpreprocessは、これらのディレクトリを自動生成します。

`PREFIX`を`/usr` とした場合、インストールにスーパーユーザ権限が求められます。
`PREFIX`を`/usr` でないものにした場合、環境変数`PATH`および`LD_LIBRARY_PATH`の修正が必要になります。
これについては[上の記載](#スクリプトによるインストール)をご参照下さい。


## スクリプトdownload

ライブラリのソースコード一式はGitHubに置いてあります。
これを取得する方法は幾つかありますが、installスクリプトで三番目に実行されるdownloadでは、次の4つの方法を選べます。

 - GitHub最新リポジトリからmainブランチのZIPファイルをダウンロードし解凍
 - GitHub最新リポジトリmainブランチをクローン（SSH）
 - GitHub最新リポジトリmainブランチをクローン（HTTPS）
 - GitHub最新リポジトリmainブランチをpull

gitを使いたくないという人は、最初のZIPファイルダウンロードのみが選択肢となります。
デフォルトでもこれを選択するようにしています。
```sh
% ./scripts/download
```
とすれば、全ライブラリがサブディレクトリ以下に展開されます。

最新リポジトリをクローンして使いたいという人は、SSHならば
```sh
% ./scripts/download
```
HTTPSならば
```sh
% ./scripts/download git-https
```
として下さい。
どちらを使うかについては、ご自分の環境設定をご確認下さい。

最新バージョンにキープアップしたいという場合は
```sh
% ./scripts/download git-pull
```
とすれば、全てのライブラリについて最新リポジトリのmainブランチをpullします。
これはライブラリ開発に参加したい人向けです。


## スクリプトbuild

mi-libのソース一式が展開され、環境変数も設定済ならば、ビルド可能です。
全ライブラリを一括ビルドするスクリプトがbuildで、スクリプトinstallの中では最後に実行されます。

ビルドの方法は次から選べます。

 - debパッケージを生成しインストール
 - Cライブラリとしてビルド＆インストール
 - Cライブラリとしてビルド＆インストール（デバッグモード）
 - C++ライブラリとしてビルド＆インストール

慣れない人は、最初のdebパッケージインストールを選択するのが楽ですが、スーパユーザ権限が必要です。
```sh
% build deb
```
で、ソースコンパイルからdpkgインストールまで行います。
ここで作られるライブラリはC用のものです。
また、インストール先は/usr 以下となります。
`PREFIX`は無視されるのでご注意下さい。

システムディレクトリを汚したくない、開発用ディレクトリは自分で用意したい、という人は、普通にCライブラリをビルド＆インストールするのが良いでしょう。
```sh
% build
```
とすれば完了します。
また、
```sh
% build debug
```
とすれば、コンパイルする際に-gオプションと-pgオプションが追加されます。
これはライブラリ開発者向けです。

まだ開発中ですが、C++用のライブラリとしてビルドすることも可能です。
これは
```sh
% build cpp
```
で行えます。
コンパイラにはg++を使用します。
規格はC++17準拠です。

g++以外のコンパイラでC++ライブラリをビルドしたい、という方は、恐らくスクリプトの中身を読める方だと思うので、ご自由に改造して下さい。


## ライブラリの個別コンパイル＆インストール

それぞれのライブラリのディレクトリにあるconfig.orgファイルをconfigという名前でコピーし編集することで、個別に設定を変えることができます。
[install\_dependencies](##スクリプトinstall\_dependencies)のところに書いた、サードパーティ製ライブラリの使用/不使用の選択も、これで行います。

たとえばzeda/configを見ると
```
# XML parser (libxml2)
CONFIG_USE_LIBXML=y
```
という箇所があります。
これはlibxml2の使用を有効化するという意味です。
yをnに変えれば、libxml2がなくてもビルド・リンクできるようになります。
その代わり、（当然ですが）XMLパーサを必要とする機能（RoKiのURDF読み込みなど）は全て無効化されます。

ディレクトリ下にconfigが無い場合、ビルドプロセス内でconfig.orgが自動的にコピーされます。

ライブラリのビルド＆インストールは
```sh
% make && make install
```
で完了します。


## ユーティリティツールについて

各ライブラリのユーティリティツールには二種類あります。
一つめは、ライブラリを使ったプログラミングに必要なツール群で、toolsディレクトリの下にあります。
もう一つは、ライブラリの機能を使った簡易アプリケーションで、appディレクトリ下にあります。
どちらも `${PREFIX}`/binの下にインストールされます。

前者は開発に必須です。
後者は必須ではありませんが、ライブラリの機能を簡単に体験できます。
サンプルコードとしてご参照頂くのにも良いと思います。
