import matplotlib.pyplot as plt
import pandas as pd
from tracetools_analysis.loading import load_file
import matplotlib.cm as cm
import numpy as np

# ==========================================
# 設定
# ==========================================
ZOOM_RANGE = [10.0, 11.0] 
# ==========================================

path = '/home/kameusagi/.ros/tracing/my_tracing_session/ust/converted'

try:
    events = load_file(path)
    df = pd.DataFrame(events)

    if df.empty:
        print("エラー: データがありません。")
    else:
        time_col = '_timestamp' if '_timestamp' in df.columns else 'timestamp'
        name_col = '_name' if '_name' in df.columns else 'event_name'
        tid_col = next(c for c in ['_tid', 'tid', 'vtid'] if c in df.columns)
        
        min_ts = df[time_col].min()
        df['elapsed_time'] = (df[time_col] - min_ts) / 1e9
        
        mask = (df['elapsed_time'] >= ZOOM_RANGE[0]) & (df['elapsed_time'] <= ZOOM_RANGE[1])
        df_zoom = df[mask].copy()

        if df_zoom.empty:
            print(f"警告: 範囲内にイベントがありません。")
        else:
            plt.figure(figsize=(16, 12))
            
            # イベント名ごとに色を固定
            unique_events = sorted(df_zoom[name_col].unique())
            color_map = cm.get_cmap('tab10', len(unique_events))
            event_to_color = {name: color_map(i) for i, name in enumerate(unique_events)}

            # スレッドごとにプロット
            unique_tids = sorted(df_zoom[tid_col].unique())
            
            for tid in unique_tids:
                tid_group = df_zoom[df_zoom[tid_col] == tid]
                # Y軸のラベル作成（実績タスク名入り）
                top_ev = tid_group[name_col].value_counts().index[:2].tolist()
                y_label = f"TID {tid}\n({', '.join(top_ev)})"
                
                # そのスレッド内のイベントごとにプロット
                for name in unique_events:
                    ev_group = tid_group[tid_group[name_col] == name]
                    if not ev_group.empty:
                        plt.scatter(ev_group['elapsed_time'], 
                                    [y_label] * len(ev_group), 
                                    s=80, 
                                    color=event_to_color[name], 
                                    alpha=0.7, 
                                    label=f"{name} (TID {tid})")

            plt.title(f'Multi-thread Task Analysis: {ZOOM_RANGE[0]}s - {ZOOM_RANGE[1]}s')
            plt.xlabel('Time (seconds)')
            plt.ylabel('Executing Thread & Roles')
            plt.xlim(ZOOM_RANGE)
            plt.grid(True, linestyle=':', alpha=0.6)
            
            # 凡例をアルファベット順に整理して表示
            handles, labels = plt.gca().get_legend_handles_labels()
            # 重複は元々ないはずだが、念のため整理
            by_label = dict(zip(labels, handles))
            sorted_labels = sorted(by_label.keys())
            plt.legend([by_label[l] for l in sorted_labels], sorted_labels, 
                       loc='upper left', bbox_to_anchor=(1, 1), title="Task per Thread")
            
            plt.tight_layout()
            plt.savefig('trace_task_complete.png')
            print("成功！ 'trace_task_complete.png' を作成しました。")

except Exception as e:
    print(f"解析エラー: {e}")