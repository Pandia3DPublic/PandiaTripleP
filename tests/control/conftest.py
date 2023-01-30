import pytest
import sys
sys.path.append('../../PandiaControl')
from core import create_app

@pytest.fixture()
def app():
    app = create_app('test')
    # print("in create app")

    # other setup can go here
    yield app
    # clean up / reset resources here


@pytest.fixture()
def client(app):
    return app.test_client()
    
