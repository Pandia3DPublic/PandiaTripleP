class Config():
    SECRET_KEY = 'secret_key'
    DEBUG = False

class TestConfig(Config):
    TESTING = True
    LOGIN_DISABLED = True

class DevConfig(Config):
    DEBUG = True

config = {
    'default': Config,
    'test': TestConfig,
    'dev': DevConfig,
}