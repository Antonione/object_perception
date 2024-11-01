from setuptools import find_packages, setup

package_name = 'object_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Usa find_packages para incluir automaticamente pacotes no diretório
    install_requires=[
        'setuptools', 
        'rclpy', 
        'torch', 
        'opencv-python', 
        'cv_bridge', 
        'numpy'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/perception.launch.py']),  # Inclui arquivos de lançamento
        #('share/' + package_name + '/config', ['config/parameters.yaml']),  # Inclui arquivos de configuração
    ],
    zip_safe=True,
    maintainer='anonione',
    maintainer_email='antonione@gmail.com',
    description='Pacote ROS 2 para percepção de objetos usando YOLOv8 e câmera RealSense D435i.',
    license='MIT',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = object_perception.object_detector:main',
            'static_tf_publisher = object_perception.static_tf_publisher:main',
        ],
    },
)
