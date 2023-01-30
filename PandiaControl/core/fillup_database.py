
from .models import User, Company
import datetime
import json
import os.path


def create_database(app, db_i):
    db_i.create_all(app=app)

    #check for database config json
    jsonPath = "core/db/db.json"
    jsonData = {}
    try:
        if os.path.isfile(jsonPath):
            with open(jsonPath, 'r') as f:
                jsonData = json.load(f)
        else:
            print(f"{jsonPath} is missing!")
    except:
        print(f"Reading {jsonPath} failed in create_database!")
    
    jsonDataValid = False
    if jsonData:
        allFound = True
        #check json for completeness:
        requiredFields = { 
            "company_name",
            "company_address",
            "company_town",
            "user_email",
            "user_name",
            "user_password",
            "user_role",
        }
        for key in requiredFields:
            if not key in jsonData:
                allFound = False
        if allFound:
            jsonDataValid = True
        else:
            print("Ignoring json in create_database as it is incomplete.")
    
    if jsonDataValid:
        company_name = jsonData["company_name"]
        company_address = jsonData["company_address"]
        company_town = jsonData["company_town"]
        user_email = jsonData["user_email"]
        user_name = jsonData["user_name"]
        user_password = jsonData["user_password"]
        user_role = jsonData["user_role"]
    else:
        company_name = "SampleName"
        company_address = "SampleAddress"
        company_town = "SampleTown"
        user_email = "admin@samplemail.com"
        user_name = "Admin"
        user_password = "password"
        user_role = "Administrator"

    try:
        comp = Company(
            name=company_name,
            adress=company_address,
            town=company_town
        )
        db_i.session.add(comp)

        admin = User(
            email=user_email,
            name=user_name,
            password=user_password,
            role=user_role,
            licenses="Enterprise",
            license_exp_date=datetime.date(2099, 12, 31),
            company=comp
        )
        db_i.session.add(admin)

        try:
            db_i.session.commit()
            if jsonDataValid:
                print("Created database successfully (json)!")
            else:
                print("Created database successfully!")
        except:
            print("Failed to commit to database")
    except Exception as e:
        print(e)
        print("Database already existed")

