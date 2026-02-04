"""
配置管理模块
用于加载和管理YOLO模型和检测参数的配置
"""

import yaml
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
CONFIG_FILE = ROOT / "config.yaml"


class ConfigManager:
    """配置管理器，用于加载和获取配置"""
    
    def __init__(self, config_file=None):
        if config_file is None:
            config_file = CONFIG_FILE
        self.config_file = Path(config_file)
        self.config = self._load_config()
        self.current_config_name = self.config.get('current_config', 'yolo26n-objv1-150')
    
    def _load_config(self):
        """从YAML文件加载配置"""
        if not self.config_file.exists():
            raise FileNotFoundError(f"配置文件不存在: {self.config_file}")
        
        with open(self.config_file, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        return config
    
    def get_current_config(self):
        """获取当前活跃配置字典"""
        configs = self.config.get('configs', {})
        current = self.config.get('current_config', 'yolo26n-objv1-150')
        
        if current not in configs:
            raise ValueError(f"配置'{current}'不存在，可用的配置: {list(configs.keys())}")
        
        return configs[current]
    
    def get_model_path(self):
        """获取模型路径"""
        return self.get_current_config()['model_path']
    
    def get_class_id(self):
        """获取目标检测类别ID"""
        return self.get_current_config()['object_class_id']
    
    def get_object_name(self):
        """获取目标对象名称"""
        return self.get_current_config()['object_name']
    
    def get_description(self):
        """获取配置描述"""
        return self.get_current_config()['description']
    
    def list_available_configs(self):
        """列出所有可用的配置"""
        configs = self.config.get('configs', {})
        return {name: config.get('description', '') for name, config in configs.items()}
    
    def set_current_config(self, config_name):
        """切换当前配置"""
        configs = self.config.get('configs', {})
        if config_name not in configs:
            raise ValueError(f"配置'{config_name}'不存在，可用的配置: {list(configs.keys())}")
        
        self.config['current_config'] = config_name
        self.current_config_name = config_name
        self._save_config()
    
    def _save_config(self):
        """保存配置到文件"""
        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.safe_dump(self.config, f, allow_unicode=True, default_flow_style=False)
    
    def print_current_config(self):
        """打印当前配置信息"""
        config = self.get_current_config()
        print(f"\n{'='*50}")
        print(f"当前配置: {self.current_config_name}")
        print(f"{'='*50}")
        for key, value in config.items():
            print(f"  {key}: {value}")
        print(f"{'='*50}\n")


# 创建全局配置管理器实例
config_manager = ConfigManager()
