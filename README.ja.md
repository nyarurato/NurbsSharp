# NURBS Sharp
![NuGet Version](https://img.shields.io/nuget/vpre/NurbsSharp)
![Build Status](https://img.shields.io/github/actions/workflow/status/nyarurato/NurbsSharp/ci.yml?branch=master)

NURBS Sharpは.NET向けの依存関係なしのNURBS（非一様有理 B-スプライン）ライブラリです。  
データ構造、評価器、トポロジー演算、入出力、テッセレーション機能を.NET標準ライブラリのみで実装しています。

[English README](./README.md)

## 目次

- [特長](#特長)
- [インストール](#インストール)
- [クイックスタート](#クイックスタート)
- [例](#例)
- [ドキュメント](#ドキュメント)
- [開発](#開発)
- [コントリビュート](#コントリビュート)
- [ライセンス](#ライセンス)

## 特長

- NURBS曲線・サーフェスの生成および評価
- トポロジー演算：次数変更、ノット挿入/削除、結合/分割
- 入出力ヘルパー：`OBJ`/`STL`出力（メッシュのみ）、`IGES`入出力（NURBSのみ）、`BMP`出力（簡易）
- 曲線・サーフェス用のテッセレーションユーティリティ
- 対応ターゲット：.NET 8、 .NET Standard 2.1

## 制限事項

- **ノットベクトル (Knot Vector)**: 現在、"Clamped" ノットベクトルのみサポートしています（多重度 = 次数 + 1）。
- **トポロジー操作**: `RemoveKnot`は未実装です。
- **IGESサポート**:
    - **インポート**: Entity Type 126 (Rational B-spline curve) と 128 (Rational B-spline surface) のみサポートしています。
    - **エクスポート**: NURBSエンティティのみサポートしています。メッシュのエクスポートはサポートされていません。
- **メッシュエクスポート (OBJ/STL)**: メッシュオブジェクトのみサポートしています。NURBSからの直接エクスポートはサポートされていません（テッセレーションが必要です）。
- **生成 (Generation)**: 近似 (Approximation) は未実装です（補間 (Interpolation) のみ利用可能）。

## インストール

NuGetからインストール

```powershell
dotnet add package NurbsSharp
```

またはプロジェクトファイルにパッケージ参照を追加してください。

## クイックスタート

基本的な使用例（C#）

```csharp
using NurbsSharp.Core;
using NurbsSharp.Geometry;
using NurbsSharp.Tesselation;
using NurbsSharp.IO;

// NURBS曲面を作成して評価
int degreeU = 3, degreeV = 3;
var knotsU = new double[] {0,0,0,0,1,1,1,1};
var knotsV = new double[] {0,0,0,0,1,1,1,1};
var kvU = new KnotVector(knotsU);
var kvV = new KnotVector(knotsV);

// コントロールポイント（u x v グリッド）
ControlPoint[][] controlPoints = new ControlPoint[4][];
controlPoints[0] = new ControlPoint[] {
    new ControlPoint(0.0, 0.0, 0.0, 1),
    new ControlPoint(1.0, 0.0, 1.0, 1),
    new ControlPoint(2.0, 0.0, 3.0, 1),
    new ControlPoint(3.0, 0.0, 3.0, 1)
};
// ... コントロールポイントの定義 ...

var surface = new NurbsSurface(degreeU, degreeV, kvU, kvV, controlPoints);
var point = surface.GetPos(0.5, 0.5);
Console.WriteLine(point);
```

## 例

NURBSサーフェスを分割メッシュ化してSTLとしてエクスポートする例

```csharp
int divideN = 20;
var mesh = SurfaceTessellator.Tessellate(surface, divideN, divideN);
using (var fs = new FileStream("test_output.stl", FileMode.Create, FileAccess.Write))
{
    await STLExporter.ExportAsync(mesh, fs);
}
```

サンプルプロジェクト: https://github.com/nyarurato/NurbsSharpSample  
Github Wiki: https://github.com/nyarurato/NurbsSharp/wiki/Samples  


## ドキュメント

APIドキュメント: https://nyarurato.github.io/NurbsSharp/

## 開発

リポジトリをクローンしてローカルでビルド

```powershell
dotnet restore
dotnet build -c Debug
```

ユニットテストを実行するには：

```powershell
dotnet test -c Debug
```

プロジェクト構成のハイライト：

- `src/Core` — 数値ヘルパーとコア型
- `src/Geometry` — `NurbsCurve`, `NurbsSurface`, `Mesh` など
- `src/Evaluation` — 評価器（曲線/サーフェス）
- `src/Operation` — トポロジー演算（degree, knot, join, split）
- `src/IO` — `OBJExporter`, `STLExporter`, IGES

## コントリビュート

貢献を歓迎します。  
典型的なワークフロー：

1. リポジトリをフォーク
2. わかりやすいブランチ名でブランチを作成
3. 必要に応じて新しいテストを追加
4. プルリクエストを `main`（またはこのリポジトリで使用されている対象ブランチ）へ送る

## ライセンス

MIT License
