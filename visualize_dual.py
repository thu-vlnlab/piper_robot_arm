import pickle
import numpy as np
import plotly.graph_objs as go
import plotly.express as px
from plotly.subplots import make_subplots
import dash
from dash import dcc, html, Input, Output, callback
from PIL import Image
import io
import base64

# 数据路径
pkl_root = '/media/nvidia/Elements/aloha_arm_only/down'
# pkl_root = '/root/autodl-tmp/aloha_arm_only_trimmed'
pkl_path = pkl_root + '/episode_52.pkl'
# pkl_path = './test.pkl'

def load_data(pkl_path):
    """加载pickle数据文件"""
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    return data

def prepare_data(data):
    """准备可视化数据"""
    # 提取时间序列数据
    qpos_data = np.array(data['/observations/qpos'])  # shape: (timesteps, 7)
    qvel_data = np.array(data['/observations/qvel'])  # shape: (timesteps, 6)
    effort_data = np.array(data['/observations/effort'])  # shape: (timesteps, 7)
    
    # 图像数据（所有帧）
    cam_main_images = data['/observations/images/cam_main']  # list of frames
    cam_second_images = data['/observations/images/cam_second']  # list of frames
    
    return {
        'qpos': qpos_data,
        'qvel': qvel_data, 
        'effort': effort_data,
        'cam_main_images': cam_main_images,
        'cam_second_images': cam_second_images,
        'timesteps': range(len(qpos_data))
    }

def image_to_base64(image_array):
    """将图像数组转换为base64编码"""
    # 确保图像数据类型正确
    if image_array.dtype != np.uint8:
        image_array = (image_array * 255).astype(np.uint8)
    
    # 转换RGB到PIL Image
    pil_image = Image.fromarray(image_array)
    
    # 保存为base64
    buffer = io.BytesIO()
    pil_image.save(buffer, format='PNG')
    img_str = base64.b64encode(buffer.getvalue()).decode()
    return f"data:image/png;base64,{img_str}"

def create_sample_data():
    """创建示例数据用于演示"""
    timesteps = 50
    
    # 生成示例关节数据
    qpos_data = np.random.randn(timesteps, 7) * 0.5  # 7个关节
    qvel_data = np.random.randn(timesteps, 6) * 0.1  # 6个关节速度
    effort_data = np.random.randn(timesteps, 7) * 2  # 7个关节力矩
    
    # 添加一些趋势使数据更真实
    t = np.linspace(0, 4*np.pi, timesteps)
    for i in range(7):
        qpos_data[:, i] = np.sin(t + i*0.5) + 0.1 * np.random.randn(timesteps)
        effort_data[:, i] = 0.5 * np.cos(t + i*0.3) + 0.1 * np.random.randn(timesteps)
    
    for i in range(6):
        qvel_data[:, i] = np.cos(t + i*0.7) * 0.2 + 0.05 * np.random.randn(timesteps)
    
    # 创建多帧示例摄像头图像 (每帧480x640x3)
    cam_main_images = []
    cam_second_images = []
    
    for frame_idx in range(timesteps):
        # 主摄像头图像 - 创建动态的圆形效果
        img1 = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 添加一些几何图形，随时间变化
        center_x = 320 + 50 * np.sin(frame_idx * 0.1)
        center_y = 240 + 50 * np.cos(frame_idx * 0.1)
        
        # 画一个圆
        for y in range(480):
            for x in range(640):
                dist = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                if dist < 50:
                    img1[y, x] = [255, 100, 100]  # 红色圆
                elif dist < 100:
                    img1[y, x] = [100, 255, 100]  # 绿色环
                elif dist < 150:
                    img1[y, x] = [100, 100, 255]  # 蓝色环
        
        # 添加一些随机噪声
        img1 = img1 + np.random.randint(0, 30, (480, 640, 3), dtype=np.uint8)
        img1 = np.clip(img1, 0, 255)
        
        cam_main_images.append(img1)
        
        # 第二个摄像头图像 - 创建不同的动态效果
        img2 = np.zeros((720, 1280, 3), dtype=np.uint8)
        
        # 创建不同尺寸的图像（模拟cam_second的尺寸）
        center_x2 = 640 + 100 * np.cos(frame_idx * 0.08)
        center_y2 = 360 + 80 * np.sin(frame_idx * 0.12)
        
        # 画多个圆环
        for y in range(720):
            for x in range(1280):
                dist2 = np.sqrt((x - center_x2)**2 + (y - center_y2)**2)
                if dist2 < 60:
                    img2[y, x] = [255, 255, 100]  # 黄色圆心
                elif dist2 < 120:
                    img2[y, x] = [100, 255, 255]  # 青色内环
                elif dist2 < 180:
                    img2[y, x] = [255, 100, 255]  # 紫色中环
                elif dist2 < 240:
                    img2[y, x] = [100, 255, 100]  # 绿色外环
        
        # 添加一些随机噪声
        img2 = img2 + np.random.randint(0, 25, (720, 1280, 3), dtype=np.uint8)
        img2 = np.clip(img2, 0, 255)
        
        cam_second_images.append(img2)
    
    return {
        'qpos': qpos_data,
        'qvel': qvel_data, 
        'effort': effort_data,
        'cam_main_images': cam_main_images,
        'timesteps': range(timesteps)
    }


# 加载数据
data = load_data(pkl_path)
viz_data = prepare_data(data)

# 创建Dash应用
app = dash.Dash(__name__)

# 关节名称
joint_names_qpos = [f'Joint_{i+1}' for i in range(7)]
joint_names_qvel = [f'Joint_{i+1}' for i in range(6)]
joint_names_effort = [f'Joint_{i+1}' for i in range(7)]

# 布局
app.layout = html.Div([
    html.H1("机器人数据可视化", style={'textAlign': 'center', 'marginBottom': 30}),
    
    # 图像控制区域
    html.Div([
        html.H3("摄像头图像 (cam_main & cam_second)"),
        
        # 时间步slider
        html.Div([
            html.Label("选择时间步:"),
            dcc.Slider(
                id='frame-slider',
                min=0,
                max=len(viz_data['timesteps']) - 1,
                value=0,
                step=1,
                marks={i: str(i) for i in range(0, len(viz_data['timesteps']), max(1, len(viz_data['timesteps'])//10))},
                tooltip={"placement": "bottom", "always_visible": True}
            )
        ], style={'marginBottom': 20}),
        
        # 当前帧信息
        html.Div(id='frame-info', style={'marginBottom': 10}),
        
        # 双摄像头图像显示
        html.Div([
            html.Div([
                html.H4("主摄像头 (cam_main)"),
                html.Img(id='cam-main-image', 
                        style={'width': '100%', 'height': 'auto', 'maxWidth': '500px', 'border': '1px solid #ccc'})
            ], style={'width': '48%', 'display': 'inline-block', 'verticalAlign': 'top', 'marginRight': '4%'}),
            
            html.Div([
                html.H4("第二摄像头 (cam_second)"),
                html.Img(id='cam-second-image', 
                        style={'width': '100%', 'height': 'auto', 'maxWidth': '600px', 'border': '1px solid #ccc'})
            ], style={'width': '48%', 'display': 'inline-block', 'verticalAlign': 'top'})
        ])
    ], style={'marginBottom': 30}),
    
    # 数据图表区域
    html.Div([
        html.Div([
            html.H3("关节位置 (QPOS)"),
            dcc.Graph(id='qpos-plot')
        ], style={'width': '48%', 'display': 'inline-block', 'verticalAlign': 'top', 'marginRight': '4%'}),
        
        html.Div([
            html.H3("关节速度 (QVEL)"),
            dcc.Graph(id='qvel-plot')
        ], style={'width': '48%', 'display': 'inline-block', 'verticalAlign': 'top'})
    ], style={'marginBottom': 30}),
    
    html.Div([
        html.Div([
            html.H3("关节力矩 (EFFORT)"),
            dcc.Graph(id='effort-plot')
        ], style={'width': '100%', 'display': 'inline-block', 'verticalAlign': 'top'})
    ])
])

# 回调函数：更新图像显示
@app.callback(
    [Output('cam-main-image', 'src'),
     Output('cam-second-image', 'src'),
     Output('frame-info', 'children')],
    Input('frame-slider', 'value')
)
def update_images(selected_frame):
    """根据滑块选择的帧更新两个摄像头的图像显示"""
    if selected_frame >= len(viz_data['cam_main_images']):
        selected_frame = 0
    
    # 获取两个摄像头的图像
    main_image = viz_data['cam_main_images'][selected_frame]
    second_image = viz_data['cam_second_images'][selected_frame]
    
    # 转换为base64
    main_image_src = image_to_base64(main_image)
    second_image_src = image_to_base64(second_image)
    
    # 更新帧信息
    info_text = f"当前帧: {selected_frame} / {len(viz_data['cam_main_images']) - 1}"
    
    return main_image_src, second_image_src, info_text

# 回调函数：QPOS图表
@app.callback(
    Output('qpos-plot', 'figure'),
    Input('frame-slider', 'value')
)
def update_qpos_plot(selected_frame):
    fig = go.Figure()
    
    # 添加当前时间步的垂直线
    fig.add_vline(x=selected_frame, line_dash="dash", line_color="red", 
                  annotation_text=f"当前时间步: {selected_frame}")
    
    for i, joint_name in enumerate(joint_names_qpos):
        fig.add_trace(go.Scatter(
            x=list(viz_data['timesteps']),
            y=viz_data['qpos'][:, i],
            mode='lines+markers',
            name=joint_name,
            line=dict(width=2)
        ))
    
    fig.update_layout(
        title='关节位置随时间变化',
        xaxis_title='时间步',
        yaxis_title='关节位置 (rad)',
        hovermode='x unified',
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    )
    
    return fig

# 回调函数：QVEL图表
@app.callback(
    Output('qvel-plot', 'figure'),
    Input('frame-slider', 'value')
)
def update_qvel_plot(selected_frame):
    fig = go.Figure()
    
    # 添加当前时间步的垂直线
    fig.add_vline(x=selected_frame, line_dash="dash", line_color="red", 
                  annotation_text=f"当前时间步: {selected_frame}")
    
    for i, joint_name in enumerate(joint_names_qvel):
        fig.add_trace(go.Scatter(
            x=list(viz_data['timesteps']),
            y=viz_data['qvel'][:, i],
            mode='lines+markers',
            name=joint_name,
            line=dict(width=2)
        ))
    
    fig.update_layout(
        title='关节速度随时间变化',
        xaxis_title='时间步',
        yaxis_title='关节速度 (rad/s)',
        hovermode='x unified',
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    )
    
    return fig

# 回调函数：EFFORT图表
@app.callback(
    Output('effort-plot', 'figure'),
    Input('frame-slider', 'value')
)
def update_effort_plot(selected_frame):
    fig = go.Figure()
    
    # 添加当前时间步的垂直线
    fig.add_vline(x=selected_frame, line_dash="dash", line_color="red", 
                  annotation_text=f"当前时间步: {selected_frame}")
    
    for i, joint_name in enumerate(joint_names_effort):
        fig.add_trace(go.Scatter(
            x=list(viz_data['timesteps']),
            y=viz_data['effort'][:, i],
            mode='lines+markers',
            name=joint_name,
            line=dict(width=2)
        ))
    
    fig.update_layout(
        title='关节力矩随时间变化',
        xaxis_title='时间步',
        yaxis_title='关节力矩 (Nm)',
        hovermode='x unified',
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    )
    
    return fig

if __name__ == '__main__':
    print("启动可视化服务器...")
    print("访问 http://127.0.0.1:6008 查看可视化结果")
    app.run(debug=True, host='0.0.0.0', port=6008)