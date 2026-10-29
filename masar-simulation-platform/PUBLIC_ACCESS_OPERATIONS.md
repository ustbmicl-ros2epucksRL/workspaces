# MASAR 固定域名公网访问架构与运维手册

> 文档状态：当前部署实况  
> 最后核对：2026-10-02  
> 公网入口：<https://sim.zzjc-ustb.cfd/>  
> 适用人员：平台维护者、服务器管理员和故障值守人员

## 1. 结论先行

MASAR 平台仍实际运行在校园服务器 `202.204.62.208` 上。外网服务器
`183.237.172.243` 不运行 MASAR 业务，只承担“Cloudflare 接入点 + SSH
中继”两个角色。

正式公网链路如下：

```text
外部访客浏览器
    │ HTTPS / WSS
    ▼
Cloudflare DNS、TLS 与边缘网络
    │ Cloudflare Tunnel（QUIC，外网服务器主动出站连接）
    ▼
外网中继服务器 183.237.172.243
    │ cloudflared → http://127.0.0.1:18090
    │                    ▲
    │                    │ SSH -L 本地端口转发
    └────────────────────┼──────────────────────────────┐
                         │                              │
                         └── SSH/TCP 22 ───────────────┘
                                                        ▼
校园服务器 202.204.62.208:127.0.0.1:18090
    │ Caddy 网关
    ├── 前端静态页面
    ├── MASAR API 与事件 WebSocket
    ├── ROS1 / ROS2 WebSocket 与视频
    └── Gazebo / RViz / Webots noVNC
```

几个关键结论：

- `sim.zzjc-ustb.cfd` 是固定地址，不使用 Pinggy 的临时域名。
- 正式链路不依赖维护者个人电脑，不要求个人电脑或 Clash Verge 保持运行。
- 正式链路不依赖校园服务器的 `127.0.0.1:7897` 代理。该代理只用于校内机器测试公网域名，以及仍保留的 Pinggy 备用容器。
- 两台服务器都不需要向公网开放 MASAR 的 `18090` 端口。该端口在两台服务器上都只绑定 `127.0.0.1`。
- Cloudflare Tunnel 由外网服务器主动连接 Cloudflare，因此外网服务器也不需要为了该域名开放入站 `80/443`。
- 当前部署是单校园源站、单外网中继。它能自动从常见进程崩溃和短暂断网中恢复，但不是双机高可用架构。

## 2. 各层的职责

| 层 | 所在位置 | 当前组件 | 职责 |
| --- | --- | --- | --- |
| 公网域名和证书 | Cloudflare | DNS、边缘 TLS | 为 `sim.zzjc-ustb.cfd` 提供解析、HTTPS 证书和公网入口 |
| Cloudflare 隧道 | 外网服务器 | `masar-cloudflared.service` | 通过 QUIC 主动连接 Cloudflare，并把请求交给外网服务器的 `127.0.0.1:18090` |
| 校园网中继 | 外网服务器 | `masar-ssh-forward.service` | 把外网服务器的 `127.0.0.1:18090` 转发到校园服务器的同一地址和端口 |
| Web 网关 | 校园服务器 | Caddy 容器 | 提供前端，并按路径代理 API、ROS、视频和 noVNC |
| 业务服务 | 校园服务器 | Docker Compose 容器 | 运行 API、ROS1、ROS2、显示服务和仿真运行时 |

### 2.1 一次请求具体如何到达平台

1. 访客打开 `https://sim.zzjc-ustb.cfd/`，DNS 与 HTTPS 由 Cloudflare 处理。
2. Cloudflare 将该主机名路由到名为 `masar-simulation-platform` 的固定 Tunnel。
3. 外网服务器上的 `cloudflared` 已主动建立多条 QUIC 连接，接收请求后访问
   `http://127.0.0.1:18090`。
4. 该端口不是外网服务器本地的业务服务，而是 SSH 命令
   `-L 127.0.0.1:18090:127.0.0.1:18090` 创建的监听端口。
5. SSH 将流量加密传送到校园服务器，最终连接校园服务器上的 Caddy。
6. Caddy 返回前端文件，或根据路径把请求转交给对应的 API、ROS 或显示服务。

虽然从整体用途看它是在“把校园服务转出去”，但当前 OpenSSH 参数实际是
`-L`，所以技术名称是**外网服务器上的 SSH 本地端口转发**，不是 `ssh -R`
反向端口转发。

### 2.2 校园服务器内部路由

Caddy 的主要路由如下，完整配置见
[`deploy/caddy/Caddyfile`](deploy/caddy/Caddyfile)：

| 公网路径 | 校园服务器目标 | 用途 |
| --- | --- | --- |
| `/` | 前端静态文件 | MASAR 操作界面 |
| `/gateway-healthz` | Caddy 自身 | 最小网关健康检查 |
| `/api/*`、`/ws/*` | `127.0.0.1:18100` | 平台 API 与事件流 |
| `/ros1/*` | `127.0.0.1:18111` | ROS1 WebSocket |
| `/ros2/*` | `127.0.0.1:18112` | ROS2 WebSocket |
| `/video/ros1/*` | `127.0.0.1:18121` | ROS1 视频流 |
| `/video/ros2/*` | `127.0.0.1:18122` | ROS2 视频流 |
| `/display/gazebo/*` | `127.0.0.1:18131` | Gazebo noVNC/WebSocket |
| `/display/rviz/*` | `127.0.0.1:18132` | RViz noVNC/WebSocket |
| `/display/webots/*` | `127.0.0.1:18133` | Webots noVNC/WebSocket |

## 3. 当前部署文件和运行位置

### 3.1 校园服务器

- 用户：`zjc`
- 项目目录：`/home/zjc/workspace/masar-simulation-platform`
- Compose 配置：[`deploy/compose.yaml`](deploy/compose.yaml)
- 运维入口：[`deploy/platform.sh`](deploy/platform.sh) 和
  [`Makefile`](Makefile)
- 本地网关：`http://127.0.0.1:18090`
- 敏感运行数据：`.runtime/`，已被 Git 忽略

MASAR 的九个 Compose 服务均使用 `restart: unless-stopped`。Docker 系统服务当前
为 `enabled` 且正在运行，因此 Docker 守护进程重启或校园服务器重启后，已有容器
会尝试自动启动。

### 3.2 外网中继服务器

- 地址：`183.237.172.243`
- 用户：`zeng-jun-chao`
- 运行目录：`~/cloudflare-relay`
- 用户级 systemd 服务：
  - `masar-ssh-forward.service`
  - `masar-cloudflared.service`
- Tunnel Token：`~/cloudflare-relay/.secrets/cloudflared.token`
- SSH 专用私钥：`~/cloudflare-relay/.secrets/masar_tunnel_ed25519`
- 固定的校园服务器主机密钥：`~/cloudflare-relay/.secrets/known_hosts`
- 仓库内的服务模板：
  - [`deploy/cloudflare-relay/systemd/masar-ssh-forward.service`](deploy/cloudflare-relay/systemd/masar-ssh-forward.service)
  - [`deploy/cloudflare-relay/systemd/masar-cloudflared.service`](deploy/cloudflare-relay/systemd/masar-cloudflared.service)

两个服务都配置了 `Restart=always` 和 `RestartSec=5`。外网服务器用户已开启
systemd linger，因此退出 SSH 会话后服务继续运行；服务器重启后也不需要该用户先
登录，用户级服务即可启动。

`cloudflared` 固定使用 `--protocol quic --edge-ip-version 4`。这是因为当前外网
服务器到 Cloudflare 的 TCP/7844 不可用，而 UDP/7844 已验证可用。

## 4. 持续运行必须满足的条件

下列条件中的任何一个长期不满足，固定域名都会完全或部分不可用。

### 4.1 Cloudflare 层

- 域名 `zzjc-ustb.cfd` 继续由当前 Cloudflare 账户托管。
- `sim.zzjc-ustb.cfd` 的 Published Application Route 仍指向当前 Tunnel。
- Tunnel Token 有效，没有被删除、吊销或轮换后漏更新到外网服务器。
- Cloudflare 账户和域名没有到期、暂停或错误修改名称服务器。

### 4.2 外网中继服务器

- 服务器开机并能访问互联网。
- 能通过 IPv4 和 UDP/7844 访问 Cloudflare Tunnel 边缘节点。
- 能通过 TCP/22 访问校园服务器 `202.204.62.208`。
- `cloudflared` 二进制、Token 文件、SSH 私钥和 `known_hosts` 文件存在且权限正确。
- 两个用户级 systemd 服务保持 enabled，且 `zeng-jun-chao` 的 linger 保持开启。
- 本机 `127.0.0.1:18090` 没有被其他进程抢占。

### 4.3 校园服务器

- 服务器开机、校园网络正常，SSH 服务可被外网中继服务器连接。
- 外网中继专用公钥仍在 `/home/zjc/.ssh/authorized_keys` 中。
- 该公钥只允许转发到 `127.0.0.1:18090`，不得把限制移除后用于日常登录。
- Docker 守护进程运行，MASAR 容器存在并能启动。
- Caddy 在 `127.0.0.1:18090` 监听且 `/gateway-healthz` 返回 `ok`。
- API、ROS1、ROS2 和显示容器健康。
- 要实际启动带图形和 GPU 的仿真时，NVIDIA 驱动、`/dev/dri`、X11 socket 和
  `.Xauthority` 也必须可用。

### 4.4 不属于正式链路的条件

- 维护者个人电脑可以关机，Clash Verge 可以退出。
- 校园服务器的 `127.0.0.1:7897` 可以不运行，固定域名仍应可用。
- `public-tunnel` 是 Compose 中保留的 Pinggy 临时备用服务。它需要 7897 代理，
  且地址可能每 60 分钟变化，但它不承载 `sim.zzjc-ustb.cfd`。
- 校园服务器通过公网域名做回环测试时，校园网可能被 Doctorcom 替换证书；这不代表
  校外访客的证书或 Cloudflare Tunnel 有问题。校内优先测本地 `127.0.0.1:18090`，
  公网验收应从真正的校外网络进行。

## 5. 当前自动恢复能力

| 故障事件 | 当前是否自动恢复 | 恢复机制 | 需要注意 |
| --- | --- | --- | --- |
| 单个 MASAR 容器进程退出 | 通常可以 | Docker `restart: unless-stopped` | 容器若只是 `unhealthy` 但进程未退出，Docker 不会仅因健康检查失败而重启它 |
| Docker 守护进程重启 | 通常可以 | Docker 服务 enabled，容器重启策略生效 | 人工执行过 `docker compose down` 后容器已被删除，不能自动恢复 |
| 校园服务器重启 | 通常可以 | Docker 启动后拉起已有容器 | GPU、X11 或挂载未就绪时，个别服务可能不健康，需要人工修复后重启 |
| SSH 连接短暂断开 | 可以 | `masar-ssh-forward.service` 退出后 5 秒重试 | 校园 SSH 长期不可达、密钥失效或主机密钥变化时无法自愈 |
| `cloudflared` 崩溃 | 可以 | `masar-cloudflared.service` 退出后 5 秒重试 | Token 被吊销或 UDP/7844 长期阻断时无法自愈 |
| 外网服务器短暂断网 | 可以 | 两个 systemd 服务持续重试 | 网络恢复和重连期间域名会暂时出现 502/1033 |
| 外网服务器重启 | 可以 | enabled 用户服务 + linger | 前提是磁盘中的密钥、Token 和 unit 文件完好 |
| 两台服务器先后重启 | 通常可以 | 各自服务启动并反复重连 | 启动顺序不重要；最终两边都健康后链路恢复 |
| Cloudflare Token 轮换 | 不可以完全自动 | 必须把新 Token 写入外网服务器并重启服务 | 旧 Token 一旦失效，Tunnel 会离线 |
| 域名、DNS 或 Tunnel 路由被删除 | 不可以 | 需要在 Cloudflare 后台人工恢复 | 本地重启服务无效 |
| 正在执行的仿真遇到服务器重启 | 服务可恢复，任务不保证恢复 | 平台容器重新启动 | 中断前的 Run 通常需要在界面重新创建或启动 |

自动恢复解决的是“进程和网络连接重新建立”，不是对正在执行的仿真做断点续跑。
WebSocket 和 noVNC 连接在任意中间层重启时都会断开，服务恢复后访客通常需要刷新页面。

这里还有两种不同的重连机制：`cloudflared` 在进程仍存活时会自行重建到 Cloudflare
边缘的 QUIC 连接；如果进程退出，则由 systemd 重新启动。SSH 则通过保活参数发现死
连接，退出后由 systemd 每 5 秒重新启动。因此普通的短时网络抖动不需要人工登录处理。

## 6. 两台服务器重启后的处理

### 6.1 只重启校园服务器

正常情况下无需人工操作。建议等待 Docker 和容器启动 2～5 分钟，然后在校园服务器
执行：

```bash
cd /home/zjc/workspace/masar-simulation-platform

systemctl is-active docker
docker compose -f deploy/compose.yaml ps
curl --noproxy '*' -fsS http://127.0.0.1:18090/gateway-healthz
curl --noproxy '*' -fsS http://127.0.0.1:18090/api/v1/capabilities >/dev/null
```

期望结果：Docker 为 `active`，MASAR 核心容器为 `running/healthy`，健康接口返回
`ok`。随后从校外访问固定域名。

如果 Docker 没启动：

```bash
sudo systemctl enable --now docker
```

如果 Docker 已启动但平台不完整：

```bash
cd /home/zjc/workspace/masar-simulation-platform
make docker-doctor

# 只恢复固定域名所需的八个核心服务，不启动 Pinggy 备用隧道
docker compose -f deploy/compose.yaml up -d \
  ros1-runtime ros2-runtime ros1-web ros2-web \
  webots-guard display api caddy
```

若必须补建前端、ROS 构建产物或本地镜像，可以在 7897 代理可用且确实需要 Pinggy
备用入口时执行 `make docker-start`。当前脚本会把 `public-tunnel` 也纳入整体健康判定；
如果 7897 不可用，Pinggy 容器会失败，并可能导致 `make docker-start` 最终报告平台未
完全健康。若只有 `public-tunnel` 不健康，但本地 `/gateway-healthz`、API 和固定域名都
正常，应把它判断为 Pinggy 备用链路故障，而不是正式链路故障。

### 6.2 只重启外网中继服务器

两个用户服务应在网络就绪后自动启动。登录 `zeng-jun-chao@183.237.172.243` 后检查：

```bash
systemctl --user is-enabled masar-ssh-forward.service masar-cloudflared.service
systemctl --user is-active masar-ssh-forward.service masar-cloudflared.service
loginctl show-user "$USER" -p Linger

curl --noproxy '*' -fsS http://127.0.0.1:18090/gateway-healthz
curl --noproxy '*' -fsS http://127.0.0.1:20241/ready
```

期望结果：两个 unit 都是 `enabled/active`、`Linger=yes`，两个 URL 都成功。

如服务未恢复：

```bash
systemctl --user daemon-reload
systemctl --user enable --now masar-ssh-forward.service
systemctl --user enable --now masar-cloudflared.service
```

### 6.3 两台服务器都重启

不要求固定启动顺序：

1. 外网服务器上的 SSH 服务会每 5 秒尝试连接校园服务器。
2. 校园服务器和 SSH 服务恢复后，外网服务器的 `127.0.0.1:18090` 自动重新出现。
3. `cloudflared` 可先连接 Cloudflare；源站暂时不可达时只会返回错误，不会破坏 Tunnel。
4. 校园 Caddy 健康后，新请求自动恢复。

等待 2～5 分钟后，先执行第 6.1 节的校园服务器检查，再执行第 6.2 节的外网服务器
检查，最后从校外网络验收域名。

## 7. 分层故障定位

排障应从用户入口逐层向源站推进，避免一开始就重启所有服务。

### 7.1 第一步：确认外部症状

从手机流量、其他校外网络或可靠代理执行：

```bash
curl -fsS -o /dev/null -w 'home=%{http_code}\n' \
  https://sim.zzjc-ustb.cfd/
curl -fsS -o /dev/null -w 'gateway=%{http_code}\n' \
  https://sim.zzjc-ustb.cfd/gateway-healthz
curl -fsS -o /dev/null -w 'api=%{http_code}\n' \
  https://sim.zzjc-ustb.cfd/api/v1/capabilities
```

正常均为 `200`。只测首页不够，因为静态页面成功不代表 API、ROS 和 WebSocket 正常。

### 7.2 常见症状与优先检查项

| 症状 | 最可能故障层 | 优先动作 |
| --- | --- | --- |
| DNS 不存在或解析失败 | 域名/Cloudflare DNS | 检查域名状态、名称服务器和 Published Application Route |
| Cloudflare 1033 / Tunnel disconnected | `cloudflared` 或 Cloudflare 出站网络 | 检查 `masar-cloudflared.service`、Token、UDP/7844 |
| Cloudflare 502，Tunnel 显示在线 | SSH 转发或校园源站 | 在外网服务器 curl `127.0.0.1:18090`，再查 SSH unit |
| 外网服务器本地 `18090` 连接被拒绝 | SSH 转发未监听 | 查 `masar-ssh-forward.service` 日志和端口占用 |
| SSH 不断认证失败 | 专用密钥或 `authorized_keys` | 核对私钥权限和校园服务器授权项 |
| SSH 报 host key verification failed | 校园服务器 SSH 主机密钥变化 | 先通过可信渠道核对新指纹，再更新 `known_hosts`，不要直接关闭严格校验 |
| 首页正常但 API 失败 | Caddy 到 API 的链路 | 查校园服务器 `api`、`caddy` 容器和 18100 端口 |
| 页面正常但 ROS1/ROS2 不连接 | ROS sidecar/WebSocket | 查 `ros1-web`、`ros2-web` 及相应 runtime |
| noVNC 打开但黑屏/断开 | display、仿真器、X11/GPU | 查 `display`、对应 runtime、X11 socket 和仿真 Run |
| 校内访问公网域名出现异常证书 | Doctorcom 校园网拦截 | 用校外网络验收；校内先检查本地回环地址 |
| Pinggy 临时 URL 变化或失败 | 旧备用链路 | 不影响 `sim.zzjc-ustb.cfd`，不要误判为正式入口故障 |

### 7.3 外网中继服务器检查

```bash
# 服务状态
systemctl --user --no-pager --full status \
  masar-ssh-forward.service masar-cloudflared.service

# 最近 30 分钟日志
journalctl --user -u masar-ssh-forward.service \
  --since '30 minutes ago' --no-pager
journalctl --user -u masar-cloudflared.service \
  --since '30 minutes ago' --no-pager

# 本地监听和两级健康检查
ss -lnt | grep ':18090'
curl --noproxy '*' -fsS http://127.0.0.1:18090/gateway-healthz
curl --noproxy '*' -fsS http://127.0.0.1:20241/ready
```

恢复顺序：

```bash
systemctl --user restart masar-ssh-forward.service
curl --noproxy '*' -fsS http://127.0.0.1:18090/gateway-healthz

systemctl --user restart masar-cloudflared.service
curl --noproxy '*' -fsS http://127.0.0.1:20241/ready
```

如果 SSH unit 反复失败，不要只循环重启。日志中应进一步区分网络超时、认证失败、
主机密钥不匹配和 `Address already in use`。

### 7.4 校园服务器检查

```bash
cd /home/zjc/workspace/masar-simulation-platform

make docker-status
docker compose -f deploy/compose.yaml ps
curl --noproxy '*' -fsS http://127.0.0.1:18090/gateway-healthz
curl --noproxy '*' -fsS http://127.0.0.1:18090/api/v1/capabilities
```

查看全部日志或单个服务：

```bash
make docker-logs
make docker-logs SERVICE=caddy
make docker-logs SERVICE=api
make docker-logs SERVICE=ros1-web
make docker-logs SERVICE=ros2-web
make docker-logs SERVICE=display
```

只重启一个确认有问题的服务：

```bash
docker compose -f deploy/compose.yaml restart caddy
docker compose -f deploy/compose.yaml restart api
```

多个服务状态混乱时再执行完整恢复：

```bash
make docker-doctor
docker compose -f deploy/compose.yaml up -d \
  ros1-runtime ros2-runtime ros1-web ros2-web \
  webots-guard display api caddy
```

仅当校园服务器的 7897 代理可用且要同时恢复 Pinggy 备用入口时，才使用
`make docker-start` 恢复全部九个服务。

不要把 `make docker-stop` 当作普通重启命令。它会先终止活动 Run，再删除整个 Compose
栈；执行后必须通过 `make docker-start` 重新创建服务。

## 8. Cloudflare Tunnel 故障处理

### 8.1 QUIC 连接失败

正常日志包含类似 `Registered tunnel connection`，并显示 `protocol=quic`。如果持续出现
QUIC timeout：

1. 确认外网服务器允许 IPv4 UDP/7844 出站。
2. 确认 unit 中仍有 `--protocol quic --edge-ip-version 4`。
3. 检查服务器防火墙、上游网络策略和云厂商安全策略。
4. 网络恢复后重启 `masar-cloudflared.service`。

当前环境不要改回 `http2` 作为默认方案，因为 TCP/7844 已验证受阻。

### 8.2 Token 轮换或泄露

Token 不能提交到 Git、工单、截图或聊天。若旧 Token 暴露或在 Cloudflare 后台执行了
Rotate token，应在外网服务器交互式写入新值，避免进入 shell 历史：

```bash
read -rsp 'New Cloudflare Tunnel token: ' CF_TOKEN
echo
umask 077
printf '%s\n' "$CF_TOKEN" > ~/cloudflare-relay/.secrets/cloudflared.token
unset CF_TOKEN

systemctl --user restart masar-cloudflared.service
systemctl --user is-active masar-cloudflared.service
journalctl --user -u masar-cloudflared.service \
  --since '5 minutes ago' --no-pager
```

若当前聊天里曾出现过旧 Token，而尚未轮换，应把轮换视为待办安全事项。

## 9. SSH 中继故障处理与安全限制

### 9.1 当前自动保活参数

SSH unit 使用：

- `ServerAliveInterval=30`
- `ServerAliveCountMax=3`
- `ExitOnForwardFailure=yes`
- systemd `Restart=always`
- systemd `RestartSec=5`

因此连接失去响应约 90 秒后 SSH 会退出，systemd 随后重新建立连接。网络明确拒绝连接
时通常会更快进入重试。

### 9.2 专用密钥限制

校园服务器 `authorized_keys` 中的隧道公钥当前使用以下限制思路：

```text
restrict,port-forwarding,command="/bin/false",permitopen="127.0.0.1:18090" ...
```

它只允许该密钥把连接转发到指定回环端口，不能得到交互式 shell，也不能转发到任意
其他内网服务。不要移除 `restrict`、`command` 或 `permitopen`。

不要重新加入 `permitlisten="none"`；该选项曾与当前转发方式不兼容并导致认证/转发失败。

### 9.3 校园服务器 SSH 主机密钥变化

重装校园服务器或重新生成 SSH host key 后，严格主机密钥校验会阻止中继连接，这是安全
保护，不应使用 `StrictHostKeyChecking=no` 绕过。正确流程是：

1. 在校园服务器本机查看新 host key 指纹。
2. 通过独立可信渠道把指纹与外网服务器看到的指纹进行比对。
3. 确认一致后，更新外网服务器 `~/cloudflare-relay/.secrets/known_hosts`。
4. 重启 `masar-ssh-forward.service` 并检查本地 `18090`。

## 10. 开机自恢复配置审计

建议在系统升级、服务迁移或每季度维护时执行一次。

### 10.1 校园服务器

```bash
systemctl is-enabled docker
systemctl is-active docker

docker inspect \
  $(docker compose -f deploy/compose.yaml ps -q) \
  --format '{{.Name}} restart={{.HostConfig.RestartPolicy.Name}}'
```

期望 Docker 为 `enabled/active`，MASAR 服务的 restart policy 均为
`unless-stopped`。

### 10.2 外网中继服务器

```bash
systemctl --user is-enabled masar-ssh-forward.service masar-cloudflared.service
systemctl --user is-active masar-ssh-forward.service masar-cloudflared.service
loginctl show-user "$USER" -p Linger

stat -c '%a %n' \
  ~/cloudflare-relay/.secrets/cloudflared.token \
  ~/cloudflare-relay/.secrets/masar_tunnel_ed25519 \
  ~/cloudflare-relay/.secrets/known_hosts
```

Token 和私钥应至少为 `0600`。如果 linger 不是 `yes`，需由有权限的管理员执行：

```bash
sudo loginctl enable-linger zeng-jun-chao
```

## 11. 完整恢复后的验收

按以下顺序验收：

1. 校园服务器：`http://127.0.0.1:18090/gateway-healthz` 返回 `ok`。
2. 外网服务器：同一地址返回 `ok`，证明 SSH 转发正常。
3. 外网服务器：`http://127.0.0.1:20241/ready` 成功，证明 Tunnel 有可用连接。
4. 校外网络：固定域名首页、网关和 API 均返回 HTTP 200。
5. 浏览器：页面无失败请求或控制台错误。
6. 功能：ROS1、ROS2、事件流和 Webots WebSocket 均能连接。
7. 若发生过服务器重启，重新创建一次测试 Run；不要假设中断前 Run 会续跑。

最小公网验收：

```bash
curl -fsS https://sim.zzjc-ustb.cfd/gateway-healthz
curl -fsS https://sim.zzjc-ustb.cfd/api/v1/capabilities >/dev/null
curl -fsS https://sim.zzjc-ustb.cfd/api/v1/scenarios >/dev/null
```

## 12. 安全与可用性边界

- 当前域名是公开入口。Cloudflare Tunnel 负责安全传输，不等于业务鉴权；如果没有配置
  Cloudflare Access 或应用认证，知道域名的人可能调用控制 API。
- 若只允许指定人员使用，应在 Cloudflare Zero Trust 中为该 hostname 配置 Access
  邮箱 OTP/白名单，并重新验收同源 WebSocket。
- `.runtime/` 和 `deploy/cloudflare-relay/.secrets/` 已被 Git 忽略，但“被忽略”不等于
  “泄露后仍安全”。Token 或订阅链接泄露后仍必须轮换。
- 当前只有一个校园源站和一个中继服务器。任一服务器长期宕机时，没有第二节点接管。
- Cloudflare 边缘可用性不代表源站高可用。如果需要更高可靠性，应增加第二中继、监控
  告警和经过验证的备份恢复流程。
- Docker 的 restart policy 不会因为容器状态变为 `unhealthy` 就自动重启仍在运行的
  进程。若要覆盖“进程未退出但已经卡死”的故障，应另行部署经过验证的 systemd
  watchdog/健康监控；不要直接引入拥有 Docker socket 全权限的通用 autoheal 容器。

## 13. 建议的日常维护

- 每次更新平台后执行本地健康检查和校外完整验收。
- 定期查看外网服务器两个用户服务的失败记录和重启次数。
- 对 `https://sim.zzjc-ustb.cfd/gateway-healthz` 配置校外 HTTP 可用性监控；只有状态变化
  时告警，避免无意义通知。
- 在计划重启任一服务器前停止活动 Run；重启后按第 11 节验收。
- 记录 Cloudflare Tunnel、域名和服务器的负责人，避免账户或域名到期无人处理。
- 更换校园服务器 IP、SSH host key、域名或中继服务器时，同步更新本手册和仓库内
  systemd 模板。

## 14. 快速恢复清单

公网无法访问时，依次回答下面五个问题：

1. 校外访问域名得到的是 DNS 错误、1033、502，还是页面内某项功能失败？
2. 外网服务器的 `masar-cloudflared.service` 是否 active，metrics `/ready` 是否成功？
3. 外网服务器的 `masar-ssh-forward.service` 是否 active，本地 `18090` 是否返回 `ok`？
4. 校园服务器本地 `18090` 是否返回 `ok`，MASAR 核心容器是否 healthy？
5. 如果页面与 API 正常，具体失败的是 ROS、视频还是某个 noVNC 路由？

找到第一个答案为“否”的层，只恢复该层；恢复后再从校园服务器、外网服务器到公网入口
逐级验收。
