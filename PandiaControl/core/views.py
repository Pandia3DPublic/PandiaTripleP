from flask_login import login_required, current_user
from flask import Blueprint, render_template, request, flash, redirect, url_for, jsonify, Response, current_app
import requests
import socket
import os
from .models import db, Model, Company, System, Workhorse
import numpy as np
import traceback
from core import logger

views = Blueprint('views', __name__)  # this actually gets registered
upload_folder_path = "static/3dModels/"

@views.route('/')
@login_required
def home():
    return redirect(url_for('views.start'))

@views.route('/StartPage')
@login_required
def start():
    return render_template("Startpage.html", NavActive=1, user=current_user, current_company=current_user.company)
    

@views.route("/VolumeScanner/<SideBarMode_>/<DisplayMode_>")
@login_required
def VolumeScanner(SideBarMode_, DisplayMode_):
    if current_user.used_system:
        userCompany = Company.query.get(current_user.company_id)
        return render_template('VolumeScanner.html', NavActive=2, user=current_user, SideBarMode=SideBarMode_, DisplayMode=DisplayMode_, current_company=userCompany)
    else:
        return redirect(url_for('views.start'))


@views.route("/QualityControl/<SideBarMode_>/<DisplayMode_>")
@login_required
def QualityControl(SideBarMode_, DisplayMode_):
    if current_user.used_system:
        userCompany = Company.query.get(current_user.company_id)
        return render_template('QualityControl.html', NavActive=3, user=current_user, SideBarMode=SideBarMode_, DisplayMode=DisplayMode_, current_company=userCompany)
    else:
        return redirect(url_for('views.start'))

@views.route("/OCMI")
@login_required
def ocmi():
    return render_template('OCMI.html', NavActive=4, user=current_user)


@views.route("/help/<cont>/<int:subbtn>/<change>")
@login_required
def help(cont, subbtn, change):
    userCompany = Company.query.get(current_user.company_id)
    return render_template('help.html', NavActive=5, user=current_user, content=cont, SubBtnActive=subbtn, accchange=change, current_company=userCompany)


@views.route("/help/<cont>/<int:subbtn>/<change>", methods=['POST'])
@login_required
def change(cont, subbtn, change):
    if change == "deleteModel":
        # get input filename and filepath on disk
        requestedfileName = request.form.get("modelDropdown")
        if requestedfileName ==None:
            return render_template('help.html', NavActive=5, user=current_user, content=cont, SubBtnActive=subbtn, current_company=current_user.company)
        filepath = os.path.join(current_app.root_path, upload_folder_path, requestedfileName)
        # get company
        userCompany = Company.query.get(current_user.company_id)
        # delete 3dModel object IN DATABASE
        if (current_user.used_system.used_volumesettings.referenceModel != None and requestedfileName == current_user.used_system.used_volumesettings.referenceModel.filename):
            current_user.used_system.used_volumesettings.referenceModel = None
        model = Model.query.filter_by(filename=requestedfileName).first()
        try:
            db.session.delete(model)
            db.session.commit()
        except:
            print("Deleting model from db failed")
        # check if file exists, then delete 3dModel ON DISK
        if os.path.exists(filepath):
            os.remove(filepath)
            flash("File sucessfully removed from database and disk.", category="successFile")
        else:
            flash("File successfully removed from database.", category="successFile")
        return render_template('help.html', NavActive=5, user=current_user, content=cont, SubBtnActive=subbtn, current_company=userCompany)
    
    if change == "deleteSystem":
        requestedSystem= request.form.get("systemDropdown")
        if requestedSystem ==None:
            return render_template('help.html', NavActive=5, user=current_user, content=cont, SubBtnActive=subbtn, current_company=current_user.company)
        system= db.session.query(System).filter_by(name=requestedSystem).first()
        try:   
            r= requests.get('http://{}/isAlive'.format(system.credentials))
            if r.status_code == 200:
                workhorse= Workhorse.query.filter_by(credentials= system.credentials).first()
                workhorse.usedInSystem = False
                system.used_workhorse = None
                res = requests.get('http://{}/clearCameraList'.format(system.credentials))
                if res.status_code != 200:
                    print("Clearing camera list failed")
        except:
            print("Delete System failed")
        try:
            db.session.delete(system) #also deletes cameras and system settings (volume and quality)
            db.session.commit()
        except:
            print("Deleting system from db failed")
        flash("System successfully removed from database.", category="successSystem")
        return render_template('help.html', NavActive=5, user=current_user, content=cont, SubBtnActive=subbtn, current_company=current_user.company)
    if change == "changeEmail":
        data= request.json
        current_user.email = data['email']
        db.session.commit()
        return jsonify(Message='OK')
    
    if change == "changePW":
        data= request.json
        current_user.password = data['newPas']
        db.session.commit()
        return jsonify(Message='OK')


@views.route('/getNetworkInfo', methods=['GET'])
def getHostName():
    try:
        hostname=socket.gethostname()
        ip = socket.gethostbyname(hostname + ".local")
        return jsonify({'ip': ip, 'hostname': hostname})
    except:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)
