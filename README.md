# 4P Calibration

4P Calibration 是离线多相机及相机—IMU 标定应用，支持单相机内参、多相机联合内外参、预积分和连续时间 B 样条联合标定。

## 下载

仅从本仓库的 [Releases](../../releases) 下载完整发布。每个正式版本同时提供：

- Linux x86-64
- Linux ARM64
- Windows x86-64
- macOS ARM64
- `SHA256SUMS`

发布由内部 GitLab CI 构建。Linux x86-64 原生构建与启动检查、Linux ARM64、Windows x86-64 GNU 和 macOS ARM64 交叉构建必须全部成功，才会同时创建 GitLab 内部 Release 与 GitHub Release；两边使用同一组二进制和校验和，不发布部分成功版本。

下载后校验文件摘要：

```sh
sha256sum -c SHA256SUMS
```

macOS 可使用：

```sh
shasum -a 256 -c SHA256SUMS
```


## 运行

```sh
# Linux
./four-p-calibration-<target> --help

# Windows PowerShell
.\four-p-calibration-x86_64-pc-windows-gnu.exe --help
```

无参数启动交互式 TUI。批处理模式：

```sh
four-p-calibration-<target> --no-tui --config calibration.yaml
```

配置格式与操作说明见 [用户指南](docs/user-guide.md)。

## 发布边界

本 GitHub 仓库仅承载公开文档和完整二进制 Release，不承载应用源码或构建依赖。GitHub 自动生成的 Source code 归档只包含本公开文档仓库内容，不是应用源码。
