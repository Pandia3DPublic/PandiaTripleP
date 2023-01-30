class Config():
    DB_NAME = 'database.db'
    SQLALCHEMY_DATABASE_URI = f'sqlite:///{DB_NAME}'
    SQLALCHEMY_TRACK_MODIFICATIONS = False
    SECRET_KEY = 'secret_key'
    DEBUG = False

class TestConfig(Config):
    DB_NAME = 'database_test.db'
    SQLALCHEMY_DATABASE_URI = f'sqlite:///{DB_NAME}'
    TESTING = True
    LOGIN_DISABLED = True

class DevConfig(Config):
    DB_NAME = 'database_dev.db'
    SQLALCHEMY_DATABASE_URI = f'sqlite:///{DB_NAME}'
    DEBUG = True

config = {
    'default': Config,
    'test': TestConfig,
    'dev': DevConfig,
}