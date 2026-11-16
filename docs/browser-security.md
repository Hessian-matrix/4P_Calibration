# 浏览器安全与本地发布

本轮只修改本地工作树；不提交、不推送、不部署，等待 GPU 开发完成后统一决定发布。安全加固不等同于 Google Safe Browsing 已解除标记，也不是第三方安全认证。

## 数据与启动边界

- 页面显示项目来源、反馈入口、用途及数据处理说明。首次打开不下载计算引擎、不建立 Rayon 线程池；用户点击“加载本地计算引擎”后才准备运行环境。
- 初始化可能因 Service Worker 控制和跨源隔离刷新一次。仅用当前标签 sessionStorage 中、按部署路径区分且有效期 60 秒的启动意图续接刷新；准备完成或失败后清除，不持久保存文件授权。
- 选择文件/目录后自动解析；“开始标定”才求解。文件、图像、IMU 与结果在浏览器中处理，不上传。托管服务仍可收到静态资源请求的 IP、请求头等连接信息；不得宣传匿名访问。
- 不需要安装桌面程序/扩展、提供账号密码或关闭浏览器防护。外部项目/反馈链接使用 no-referrer 和 noopener/noreferrer；不引入第三方统计、字体或在线脚本。
- 校验完成后显示引擎 releaseId、profile 与完整内容寻址 buildId；构建哈希是追溯信息，不是签名或安全评级。

## CSP 与浏览器限制

HTML 在资源加载之前声明 CSP，Service Worker 对其返回的页面/Worker 资源附加相同策略：

```text
default-src 'none'; script-src 'self' 'wasm-unsafe-eval'; worker-src 'self' blob:; connect-src 'self' blob:; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; base-uri 'none'; form-action 'none'; object-src 'none'
```

- 禁止普通字符串 eval、内联脚本、跨站脚本/连接、插件和表单提交；`wasm-unsafe-eval` 只允许 WASM 编译，不允许普通 JS eval。
- `worker-src blob:` 与 `connect-src blob:` 保留给 wasm-bindgen-rayon 的 Blob worker 路径；不允许 Blob 脚本作为普通页面模块来源。
- 现有参数表单、图表和 Three.js 使用动态样式，故仅样式保留 `unsafe-inline`，不是脚本许可。
- 同源脚本拥有页面数据访问权，CSP 不能保护已经被恶意同源脚本接管的页面；不得添加不可信第三方脚本。
- GitHub Pages 的首个网络响应仍由托管平台决定。meta CSP 不支持 `frame-ancestors`，Service Worker 后续响应也不能追溯保护首次加载。需要完整响应头防护时，选择可设置 CSP/COOP/COEP/Referrer-Policy/X-Content-Type-Options 的静态托管；不要声称 meta CSP 覆盖所有响应头能力。

## 安全名单处置

Google 的公开状态页只能证明当时的分类状态，不能证明人工举报、代码被入侵或误报。通过 Search Console“安全问题”取得具体示例 URL，核查后再请求复核；不通过换域名、关闭防护或自动申诉掩盖问题。

- [Google Safe Browsing 状态](https://transparencyreport.google.com/safe-browsing/search?url=https%3A%2F%2Fhessian-matrix.github.io%2F4P_Calibration%2F)
- [Search Console 安全问题说明](https://support.google.com/webmasters/answer/9044101?hl=zh-Hans)

## 执行完整性

- 页面、Service Worker 与同源 manifest 是信任根，不声称能防止同源服务器、账号或整个发布链被攻破。
- 单线程和多线程都必须由当前 `fourp-coi-v2` Service Worker 控制；即使开发服务器已经提供 COOP/COEP，也不能绕过资源校验。
- Service Worker 校验 manifest 的 API、执行档、规范路径、资源角色、十进制长度和 SHA-256，并按排序后的相对路径、NUL 与哈希字节重算 buildId。资源必须在声明的内容寻址目录内；未知资源、重定向、长度/哈希不符均拒绝，不回退到未经校验的网络响应。
- 不是“先校验一次、随后盲目 import”：每次最终模块/二进制请求都重新取得字节并校验，向浏览器返回的就是该份已校验字节。覆盖 glue 动态 import、Rayon helper 静态 import、自身 fetch 与子 Worker 的 mainJS import，保留原 URL 模块关系。
- Worker 内存只缓存已校验 manifest 的资源映射，不缓存引擎文件内容。Worker 重启后从当前两份 manifest 恢复；过期且不再被 manifest 引用的构建明确拒绝，需要重新加载当前页面。
- 页面加载器还检查 Worker 注入的校验头和下载内容哈希。头部不是签名；同源根失陷时不能单独依靠它证明可信。
- 每份引擎资源必须先完整接收并校验再交付。页面进度统计的是校验后交付的资源字节，可能按资源批次增长，不是逐网络包进度；校验与重新请求有启动开销，不宣称本轮加固改善性能。

## 本地构建与发布包

已有引擎可用时执行（目标目录必须不存在）：

```sh
python3 tools/package_web.py /tmp/4p-release-candidate
```

这只生成本地静态目录，不上传、提交或切换分支。

- shell 与 Three.js 文件使用显式白名单；只复制两份 manifest 引用的引擎构建，不复制历史构建、额外 vendor、临时文件、数据集或结果。
- 验证每个资源的规范路径、角色、长度、哈希与重算 buildId；拒绝符号链接以及构建目录内未声明文件/目录。
- 对当前受支持的 wasm-bindgen 输出形态检查模块依赖闭包，包括 canonical glue/WASM、Rayon helper 静态依赖、自身 fetch 和动态 mainJS。遇到新的生成形态或依赖表达式失败退出，不能把该检查当作通用 JavaScript 解析器。
- `build_web.py` 在发布新本地 manifest 前验证生成模块；复用已有 buildId 时必须文件/目录集合及内容完全一致，不覆盖不可变构建。
- 打包比较 HTML meta 与 Service Worker 的 CSP 声明，策略漂移直接失败。将来 GPU 构建引入新资源时须同步更新白名单/闭包规则并验证，不静默放行未知脚本。

## 本轮本地验证

- Service Worker 回归 3 项通过；其中第二次 JS 请求篡改回归加强后单独重跑通过：首次字节合法，随后改变相同 URL，必须拒绝新响应。
- `python3 -m unittest discover -s tests -p 'test_web_packaging.py'`：7 项通过，覆盖同长度篡改、缺失/未声明依赖、路径穿越、额外私有文件排除、不可变目录污染和 CSP 漂移。
- 实际打包到 `/tmp/fourp-security-local-20261113/site/4P_Calibration` 成功。无自定义响应头的普通静态服务器、项目子路径下，首次进入引擎请求为 0；用户点击后完成一次隔离初始化，2 个 Rayon worker 就绪，启动意图清除。
- 相同页面用真实 WASM 读取已有合成 SQLite，分别完成 CPU-rayon 与 cpu-single 的运行；产物通过真实下载按钮下载，`calibration.yaml` 为 334 字节；正常运行页面错误为 0。
- 已有 COOP/COEP 的开发服务器也必须取得当前 Service Worker 控制，单线程初始化成功；已有 Worker 的新标签仍停留在说明页，引擎请求为 0。
- 390 px 窄屏实际 scrollWidth 为 390；独立且明确标注的合成外参测试页成功加载本站 Three.js 并渲染两相机及视角方块，没有 CSP/页面错误。
- Worker 资源响应实际包含 CSP 与 nosniff；测试跨站请求被 `connect-src` 拦截。
- 本地故障注入服务器在第一次 glue 下载后修改第二次响应：实际请求计数 2，页面进入“引擎准备未完成”，不能开始标定，未执行篡改脚本。没有绕过错误继续运行。
- 浏览器自动化在对完成计算后的隔离标签直接 reload 时发生 frame detached/导航超时；不把该工具失败算作应用刷新通过。另用已有 Worker 的新标签验证首次说明与零引擎请求。

本轮没有修改求解器或检测数值算法。上述合成运行用于加载、交互与安全策略 smoke，不替代固定 `baton_bag` 的 corelib/OpenCV 正式标定验收；该固定数据仍缺失。网页与核心包未提交、未推送、未部署。
