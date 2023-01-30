import sqlalchemy.sql.default_comparator #we need this for deployment with nuitka, hidden import
import sqlalchemy.dialects.sqlite #we need this for deployment with nuitka, hidden import
from flask import Flask
from flask_login import LoginManager
from flask_sqlalchemy import SQLAlchemy
from flask_cors import CORS
from os.path import exists
from .config import config
import logging
import sys

logger = logging.getLogger('pandia')
logger.setLevel(logging.ERROR)
serverLogger = logging.getLogger('waitress')
serverLogger.setLevel(logging.INFO)

logger.addHandler(logging.StreamHandler(sys.stdout))
serverLogger.addHandler(logging.StreamHandler(sys.stdout))

def create_app(config_name='default'):
    cfg = config.get(config_name)
    app = Flask(__name__) #name is folder name
    app.config.from_object(cfg)
    app.app_context().push()
    CORS(app)

    from .models import db, User
    db.init_app(app)
    #for initial database setup
    if not exists(app.root_path + '/' + cfg.DB_NAME):
        from .fillup_database import create_database
        create_database(app, db) #fills database with users

    login_manager = LoginManager()
    login_manager.login_view = 'auth.login' #this is where login_required redirects to if not loged in
    login_manager.init_app(app)
    
    @login_manager.user_loader #todo function unclear
    def load_user(user_id):
        return User.query.get(int(user_id))

    from .views import views
    from .auth import auth
    from .system import system
    app.register_blueprint(views, url_prefix='/')
    app.register_blueprint(auth, url_prefix='/')
    app.register_blueprint(system, url_prefix='/')

    return app


# def clear_db(app):
#     from .models import db
#     db.session.query
