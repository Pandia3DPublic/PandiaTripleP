import pytest
import sys
sys.path.append('../../PandiaControl')
from core import create_app
from core.models import User

def test_ok():
    pass

# def test_failure():
#     assert 0

def test_new_user():
    """
    GIVEN a User model
    WHEN a new User is created
    THEN check the email, hashed_password, and role fields are defined correctly
    """
    user = User(email='patkennedy79@gmail.com', password='FlaskIsAwesome')
    assert user.email == 'patkennedy79@gmail.com'
    assert user.password != 'FlaskIsAwesome'


def test_home_page_post(client):
    """
    GIVEN a Flask application configured for testing
    WHEN the '/' page is is posted to (POST)
    THEN check that a '405' status code is returned
    """
    response = client.post('/')
    assert response.status_code == 405
    assert b"Flask User Management Example!" not in response.data


def test_getDBWorkhorses():
    app = create_app('test')
    with app.test_client() as test_client:
        response = test_client.get('/getDBWorkhorses')
        assert response.status_code == 200


def test_setup(client):
    response1 = client.get('/getDBWorkhorses')
    response2 = client.get('/getDBSystems')

    assert response1.status_code == 200
    assert response2.status_code == 200
    

# def test_sendModelToWorkhorse():
#     app = create_app('test')
#     with app.test_client() as test_client:
#         response = test_client.post('/sendModelToWorkhorse')
#         assert response.status_code == 200
    