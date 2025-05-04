# utils/config_reader.py

import yaml
import logging

logger = logging.getLogger(__name__)

class ConfigReader:
    """
    Helper class to read configuration from a YAML file.
    """
    def __init__(self, config_path):
        """
        Initializes the ConfigReader with the path to the configuration file.

        Args:
            config_path (str): The path to the YAML configuration file.
        """
        self.config_path = config_path
        logger.info(f"ConfigReader initialized with path: {self.config_path}")

    def load_config(self):
        """
        Loads and parses the YAML configuration file.

        Returns:
            dict or None: A dictionary containing the configuration, or None if loading fails.
        """
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            logger.info("Configuration loaded successfully.")
            # logger.debug(f"Loaded config: {config}") # Optional: Log full config for debugging
            return config
        except FileNotFoundError:
            logger.error(f"Configuration file not found: {self.config_path}")
            return None
        except yaml.YAMLError as e:
            logger.error(f"Error parsing configuration file {self.config_path}: {e}")
            return None
        except Exception as e:
            logger.error(f"An unexpected error occurred while loading config {self.config_path}: {e}")
            return None

# Example usage (optional, for testing the reader)
if __name__ == '__main__':
    # Assuming you have a dummy_config.yaml in the same directory for testing
    reader = ConfigReader("dummy_config.yaml")
    test_config = reader.load_config()
    if test_config:
        print("Config loaded:")
        print(test_config)
    else:
        print("Failed to load config.")