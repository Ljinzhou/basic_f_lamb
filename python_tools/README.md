# Python Tools

本目录集中存放除 `flash.py` 以外的 Python 工具。

## 主要工具

- `debug_reader.py`: ST-Link/OpenOCD 调试入口，支持命令行和 PyQt 可视化界面
- `debug_backend.py`: 调试数据采集与结构解析后端
- `debug_gui.py`: PyQt 可视化界面
- `usb_vcp_host.py`: USB VCP 上位机发包与状态监听工具

## 常用命令

```bash
python python_tools/debug_reader.py --snapshot
python python_tools/debug_reader.py --monitor --interval 0.2
python python_tools/debug_reader.py --gui
python python_tools/debug_reader.py --list --keyword motor
python python_tools/usb_vcp_host.py --menu
```

## 依赖

- `PyQt5`：图形界面
- `pyserial`：USB VCP 主机工具

## 测试

```bash
python -m unittest discover -s python_tools -p "test_*.py"
```
