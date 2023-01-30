from flask import Flask
from flask_cors import CORS
import logging
from .config import config
import sys
sys.path.append('../buildcppBindings')

logger = logging.getLogger('pandia')
logger.setLevel(logging.ERROR)
serverLogger = logging.getLogger('waitress')
serverLogger.setLevel(logging.INFO)

CameraList = []

############ init singleton ########
import cppBindings as cp
VolumeSingleton = cp.VolumeSingleton()
ScanningSingleton = cp.ScanningSingleton()

def create_app(config_name='default'):
    cfg = config.get(config_name)
    app = Flask(__name__) #name is folder name
    app.config.from_object(cfg)
    app.app_context().push()
    CORS(app)

    from .base import base
    from .volume import volume
    from .threadfunctions import threadfunctions
    from .quality import quality
    from .system import system
    from .endpoints import endpoints
    app.register_blueprint(base, url_prefix='/')
    app.register_blueprint(volume, url_prefix='/')
    app.register_blueprint(threadfunctions, url_prefix='/')
    app.register_blueprint(quality, url_prefix='/')
    app.register_blueprint(system, url_prefix='/')
    app.register_blueprint(endpoints, url_prefix='/')

    return app
