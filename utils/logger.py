import os
import logging
import logging.config

base_dir = os.path.join(os.path.dirname(__file__), '..')

if not os.path.exists(os.path.join(base_dir, 'outputs')):
    os.makedirs(os.path.join(base_dir, 'outputs'))

logging.config.fileConfig(os.path.join(base_dir, 'config/logging.conf'))


# create logger
logger = logging.getLogger('file')