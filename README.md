# 路徑規劃問題

這是一個使用 Google OR-Tools 解帶時間窗的車輛路徑問題（VRPTW） 的 Python 程式

## 安裝需求
or-tools 只適用於 python3.8 到 python3.11，並需安裝必要套件：
```bash
conda create -n ortools311 python=3.11 -y
conda activate ortools311
python -m pip install --upgrade pip
pip install ortools spyder-kernels
```
1.打開 Spyder → Tools > Preferences > Python interpreter

2.勾選 Use the following interpreter

3.指到：D:\anaconda\envs\ortools311\python.exe

## 使用 Spyder 執行
1. 打開 Spyder
2. 開啟 main.py
3. 點擊綠色執行按鈕或在 Console 輸入 `%run hw.py`
