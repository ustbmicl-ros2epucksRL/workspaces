"""
使用 neat-python 原生方式加载和使用NEAT基因组

这是最简洁的方式，不需要PyTorch，只需要neat-python

依赖:
    pip install neat-python

用法:
    python load_genome_neat_native.py
"""

import pickle
import numpy as np
from neat.nn import RecurrentNetwork  # neat-python原生网络


def load_genome_package(package_path):
    """加载基因组包"""
    with open(package_path, 'rb') as f:
        return pickle.load(f)


def create_network(genome, neat_config):
    """
    使用neat-python原生方式创建网络
    
    Args:
        genome: NEAT基因组
        neat_config: NEAT配置
        
    Returns:
        neat.nn.RecurrentNetwork: 网络实例
    """
    return RecurrentNetwork.create(genome, neat_config)


def map_output_to_action(output, u_range):
    """
    将网络输出(sigmoid激活，范围[0,1])映射到动作空间
    
    Args:
        output: 网络输出列表
        u_range: 动作范围 (float表示[-u, u] 或 [min, max])
    
    Returns:
        list: 映射后的动作
    """
    if isinstance(u_range, (list, tuple)):
        u_min, u_max = u_range[0], u_range[1]
    else:
        u_min, u_max = -u_range, u_range
    
    return [o * (u_max - u_min) + u_min for o in output]


# ============================================================================
# 示例用法
# ============================================================================

def example():
    """使用neat-python原生网络的完整示例"""
    
    print("="*60)
    print("neat-python 原生网络使用示例")
    print("="*60)
    
    # 1. 加载基因组
    package_path = "extracted_genomes/flocking_genome_package_gen99.pkl"
    print(f"\n1. 加载: {package_path}")
    
    package = load_genome_package(package_path)
    genome = package['genome']
    neat_config = package['neat_config']
    
    print(f"   任务: {package['task_name']}")
    print(f"   适应度: {package['fitness']:.4f}")
    
    # 2. 创建网络
    print("\n2. 创建网络")
    net = create_network(genome, neat_config)
    
    n_inputs = len(net.input_nodes)
    n_outputs = len(net.output_nodes)
    print(f"   输入维度: {n_inputs}")
    print(f"   输出维度: {n_outputs}")
    
    # 3. 推理示例
    print("\n3. 推理示例")
    
    # 模拟观测数据
    observation = np.random.randn(n_inputs).tolist()
    
    # 每个新episode开始时重置网络状态
    net.reset()
    
    # 前向传播（返回列表，不是tensor）
    output = net.activate(observation)
    print(f"   原始输出: {output}")
    
    # 映射到动作空间
    u_range = 1.0  # 假设动作范围 [-1, 1]
    action = map_output_to_action(output, u_range)
    print(f"   动作: {action}")
    
    # 4. 模拟控制循环
    print("\n4. 模拟控制循环 (5步)")
    net.reset()
    for step in range(5):
        obs = np.random.randn(n_inputs).tolist()
        output = net.activate(obs)
        action = map_output_to_action(output, u_range)
        print(f"   Step {step+1}: action={[f'{a:.4f}' for a in action]}")
    
    print("\n" + "="*60)
    print("完成！")
    print("="*60)


if __name__ == "__main__":
    example()

