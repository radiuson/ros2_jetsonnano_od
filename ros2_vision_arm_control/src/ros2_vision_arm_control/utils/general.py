import os
import logging

def setup_logging(log_file='ros2_vision_arm_control.log'):
    logging.basicConfig(
        filename=log_file,
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    logging.info("Logging is set up.")

def load_configuration(config_file):
    if not os.path.exists(config_file):
        logging.error(f"Configuration file {config_file} not found.")
        raise FileNotFoundError(f"Configuration file {config_file} not found.")
    
    with open(config_file, 'r') as file:
        config = file.read()
    
    logging.info(f"Configuration loaded from {config_file}.")
    return config

def save_results(results, output_file):
    with open(output_file, 'w') as file:
        file.write(results)
    logging.info(f"Results saved to {output_file}.")