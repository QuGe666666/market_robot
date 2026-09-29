# Ubuntu 22.04 Fcitx5 输入法恢复说明

本文档记录 Ubuntu 22.04 上 Fcitx5 输入法的检查、恢复和验证流程。目标是让普通用户 `lh` 的桌面会话只运行 Fcitx5，使用系统 `/usr/bin` 程序，不受旧版 Fcitx4、root 输入法进程或 Conda 中 `dbus-daemon` 的干扰。

## 1. 问题现象

- `Ctrl + Space` 无反应，无法输入中文；
- `fcitx5-remote` 返回 `0`；
- Fcitx4 与 Fcitx5 同时运行；
- root 用户运行 Fcitx4；
- Conda 环境中的 `dbus-daemon` 被输入法错误使用。

## 2. 问题根因

- Fcitx4 与 Fcitx5 不应同时运行，旧进程可能抢占输入法接口；
- 桌面输入法应运行在当前普通用户的桌面 DBus 会话中，root 不应启动它；
- 输入法相关启动命令应使用系统 `/usr/bin` 下的 Fcitx5 程序；
- Conda 的 `dbus-daemon` 不应接管桌面输入法通信；
- 只有在 Fcitx5 能正常连接当前会话后，才有意义排查快捷键冲突。

## 3. 每次开机后快速恢复

```bash
bash ~/fix_fcitx5.sh
/usr/bin/fcitx5-remote
```

返回 `1` 表示 Fcitx5 正常但当前为英文状态。需要中文时执行：

```bash
/usr/bin/fcitx5-remote -o
/usr/bin/fcitx5-remote
```

第二条命令预期返回 `2`。恢复脚本应使用 Bash 和上述绝对路径，不依赖 Conda，不删除软件包、不修改机器人项目、不自动修改 sudoers，也不无差别杀死 DBus。

## 4. 手动恢复流程

先检查当前会话和进程：

```bash
whoami
ps -ef | grep -E "fcitx5|fcitx|ibus" | grep -v grep
which fcitx
which fcitx5
which fcitx5-remote
which dbus-daemon
printf 'GTK_IM_MODULE=%s\nQT_IM_MODULE=%s\nXMODIFIERS=%s\n' "$GTK_IM_MODULE" "$QT_IM_MODULE" "$XMODIFIERS"
printf 'DBUS_SESSION_BUS_ADDRESS=%s\nXDG_RUNTIME_DIR=%s\nDISPLAY=%s\nWAYLAND_DISPLAY=%s\nXDG_SESSION_TYPE=%s\n' "$DBUS_SESSION_BUS_ADDRESS" "$XDG_RUNTIME_DIR" "$DISPLAY" "$WAYLAND_DISPLAY" "$XDG_SESSION_TYPE"
im-config -m
cat ~/.xinputrc 2>/dev/null
```

确认属于当前用户的旧 Fcitx 实例后再停止：

```bash
pkill -u "$USER" fcitx 2>/dev/null
pkill -u "$USER" fcitx5 2>/dev/null
pkill -u "$USER" fcitx-config-gtk3 2>/dev/null
```

如果发现 root 的 Fcitx4，只检测并提示，不由脚本自动处理：

```text
[WARN] Detected root-owned Fcitx4 process.
Run the following manually if confirmed:
sudo kill <PID>
```

只有确认其派生的 `dbus-daemon` 使用 `/usr/share/fcitx/dbus/daemon.conf` 且属于该旧 Fcitx4 实例时，才可人工结束对应进程。禁止误杀 system bus 或当前用户的 session bus。

启动唯一的 Fcitx5 实例：

```bash
/usr/bin/fcitx5 -d
sleep 1
/usr/bin/fcitx5-remote
```

## 5. 状态码说明

| 返回值 | 含义 |
|---:|---|
| `0` | 当前桌面会话无法连接 Fcitx5 |
| `1` | Fcitx5 正常连接，但当前输入法未激活（英文状态） |
| `2` | Fcitx5 正常连接且输入法已激活，可输入中文 |

## 6. Conda 注意事项

```bash
which dbus-daemon
```

正常应优先显示 `/usr/bin/dbus-daemon`。若显示 `/home/lh/miniconda3/bin/dbus-daemon`，说明当前 shell 受到 Conda PATH 影响。不要删除 Miniconda，也不要修改 Python 或机器人项目环境；输入法相关命令始终使用 `/usr/bin/fcitx5` 和 `/usr/bin/fcitx5-remote`。

## 7. 快捷键故障排查

确认 Fcitx5 已返回 `1` 或 `2` 后检查 GNOME 快捷键：

```bash
gsettings get org.gnome.desktop.wm.keybindings switch-input-source
gsettings get org.gnome.desktop.wm.keybindings switch-input-source-backward
```

在 `fcitx5-config-qt` 的“全局选项”中确认 `Trigger Input Method` 为 `Ctrl+Space`。若 GNOME 占用该组合键，只修改输入源切换相关快捷键，不要未经确认删除用户的全部快捷键。

## 8. Fcitx5 输入法配置

运行 `fcitx5-config-qt`，推荐顺序为：

```text
Keyboard - English (US)
Pinyin
```

不要将 `Keyboard - 汉语` 作为默认第一项。确认 `fcitx5-chinese-addons`、`fcitx5-pinyin` 已安装，并使用 `Ctrl + Space` 切换。

## 9. Wayland / Xorg

```bash
echo "$XDG_SESSION_TYPE"
```

如果 Wayland 下异常，可注销并临时选择 `Ubuntu on Xorg` 对照测试，但不要默认永久切换到 Xorg。

## 10. 永久修复建议

```bash
im-config -m
cat ~/.xinputrc
```

如果仍指向 `fcitx` 而不是 `fcitx5`，运行 `im-config` 选择 `fcitx5`，然后注销并重新登录。不要只依赖每次开机手动运行脚本。

## 11. 最终验证

```bash
ps -ef | grep -E "fcitx5|fcitx|ibus" | grep -v grep
/usr/bin/fcitx5-remote
which dbus-daemon
im-config -m
cat ~/.xinputrc
```

最终目标：普通用户 `lh` 只运行 Fcitx5；不存在 root Fcitx4 和用户 Fcitx4 冲突；`fcitx5-remote` 返回 `1` 或 `2`；拼音可用且 `Ctrl + Space` 可以切换；Conda 不影响 Fcitx5 的桌面 DBus 通信。

## 12. 安全边界与回滚

本方案不删除 IBus、Fcitx 软件包或 Miniconda，不修改机器人项目，不修改 sudoers，不误杀系统 DBus。所有配置修改前应先检查并保留原文件；如需恢复用户配置，应使用修改前的备份文件还原，并重新注销登录验证。
