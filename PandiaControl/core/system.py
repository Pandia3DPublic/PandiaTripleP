from flask import Blueprint, request, jsonify, Response, current_app, flash
from flask_login import login_required, current_user
import open3d as o3d
import numpy as np
import requests
from .models import db, Model, User, Company, System, Camera, Workhorse
from .views import upload_folder_path
import os.path
import traceback
from core import logger
from werkzeug.utils import secure_filename
import sys
#freecad imports
FREECADPATH = '/usr/lib/freecad-python3/lib' # path to your FreeCAD.so or FreeCAD.dll file
sys.path.append(FREECADPATH)
try:
    import FreeCAD # tested with v0.19
    import Mesh
    import Part
except:
    print("WARNING: FreeCAD module import failed! Please make sure to install FreeCAD 0.19 or higher.")
#end freecad imports

system = Blueprint('system', __name__)  # this actually gets registered

#supported formats
meshFormats = {'stl', 'obj', 'off', 'gltf'}
pcdFormats = {'xyz', 'xyzn', 'xyzrgb', 'pts', 'pcd'}
sharedFormats = {'ply'} #formats that could be both mesh or pcd
specialFormats = {'stp', 'step'} #mesh formats that are not o3d supported and will be imported with freecad

@system.route('/getDBWorkhorses')
@login_required
def getDBWorkhorses():
    try:
        dicts = []
        for wh in Workhorse.query.all():
            dicts.append(wh.as_dict())
        return jsonify(dicts)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/uploadReferenceModelFile", methods=["POST"])
@login_required
def uploadReferenceModelFile():
    try:
        userCompany = Company.query.get(current_user.company_id)
        uploadedFile = request.files['referenceModel']
        if (uploadedFile.filename == ''):
            return jsonify({"filename": "500"})

        newFilename = secure_filename(uploadedFile.filename)
        for model in userCompany.models: #check for duplicate
            if (model.filename == newFilename):
                return jsonify({"filename": model.filename}) #fill response data with the duplicant filename

        newModel = Model(filename=newFilename, company_id=userCompany.id) #create new db instance
        modelPath=os.path.join(current_app.root_path, upload_folder_path, newModel.filename)
        uploadedFile.save(modelPath) #write model file to disk

        try:
            fileFormat = modelPath.lower().rsplit('.', 1)[-1]
        except:
            return jsonify({"filename": "500"})

        if fileFormat in meshFormats:
            mesh = o3d.io.read_triangle_mesh(modelPath)
            if mesh.is_empty():
                os.remove(modelPath) #delete broken mesh from disk
                return jsonify({"filename": "501"})
            #center the model and write to disk
            mesh.translate(-mesh.get_center())
            if not mesh.has_triangle_normals():
                mesh.compute_triangle_normals()
            success = o3d.io.write_triangle_mesh(modelPath, mesh)
            if not success:
                print("Error: Writing mesh to disk failed.")
                return jsonify({"filename": "501"})
                
        elif fileFormat in specialFormats:
            # specialFormats are exported to static folder as obj file and named like e.g. ExampleMesh.stp.obj
            # see https://github.com/marcofariasmx/STP-STEP-to-STL-Python-Converter/blob/main/STP-to-STL.py
            try:
                shape = Part.Shape()
                shape.read(modelPath)
                os.remove(modelPath) #delete uploaded original stp file as it's no longer needed
                doc = FreeCAD.newDocument()
                pf = doc.addObject("Part::Feature","MyShape")
                pf.Shape = shape
                modelPath = modelPath + ".obj"
                Mesh.export([pf], modelPath)
            except:
                print("Exception caught while converting " + fileFormat + " file")
                return jsonify({"filename": "502"})

            mesh = o3d.io.read_triangle_mesh(modelPath)
            if mesh.is_empty():
                os.remove(modelPath) #delete broken mesh from disk
                return jsonify({"filename": "501"})
            #center the model and write to disk
            mesh.translate(-mesh.get_center())
            if not mesh.has_triangle_normals():
                mesh.compute_triangle_normals()
            success = o3d.io.write_triangle_mesh(modelPath, mesh)
            if not success:
                print("Error: Writing mesh to disk failed.")
                return jsonify({"filename": "501"})

        elif fileFormat in pcdFormats:
            pcd = o3d.io.read_point_cloud(modelPath)
            if pcd.is_empty():
                os.remove(modelPath)
                return jsonify({"filename": "501"})
            #center the model and write to disk
            pcd.translate(-pcd.get_center())
            success = o3d.io.write_point_cloud(modelPath, pcd)
            if not success:
                print("Error: Writing pcd to disk failed.")
                return jsonify({"filename": "501"})
        elif fileFormat in sharedFormats:
            isMesh = True
            model = o3d.io.read_triangle_mesh(modelPath)
            if model.is_empty():
                isMesh = False
            if not isMesh:
                model = o3d.io.read_point_cloud(modelPath)
            if model.is_empty():
                os.remove(modelPath)
                return jsonify({"filename": "501"})
            #center the model and write to disk
            model.translate(-model.get_center())
            if isMesh:
                if not mesh.has_triangle_normals():
                    mesh.compute_triangle_normals()
                success = o3d.io.write_triangle_mesh(modelPath, model)
            else:
                success = o3d.io.write_point_cloud(modelPath, model)
            if not success:
                print("Error: Writing model to disk failed.")
                return jsonify({"filename": "501"})
        else:
            print("Unknown file format in uploadReferenceModelFile!")
            return jsonify({"filename": "501"})

        #commit to database
        db.session.add(newModel)
        db.session.commit()
        return jsonify({"filename": newModel.filename}) #return filename of new model
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/saveReferenceScan", methods=["POST"])
@login_required
def saveReferenceScan():
    try:
        pcdName = request.get_json()['name']
        system = current_user.used_system
        res = requests.get("http://{}/getReferenceScan".format(system.credentials))
        if not res.ok:
            return jsonify(Message='Request getReferenceScan failed.')
        json = res.json()
        if 'Message' in json and json['Message'] == 'Failed':
            return jsonify(Message='getReferenceScan failed.')

        pcd = o3d.geometry.PointCloud()
        points = np.asarray(json['points'])
        points = points.reshape(int(np.size(points)/3), 3)
        pcd.points = o3d.utility.Vector3dVector(points)
        colors = np.asarray(json['colors'])
        colors = colors.reshape(int(np.size(colors)/3), 3)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        pcd.translate(-pcd.get_center())
        
        #pcd name
        folder = os.path.join(current_app.root_path, upload_folder_path)
        if pcdName == '':
            pcdName = 'ReferenceScan'
        pcdName = secure_filename(pcdName)
        if os.path.exists(folder + pcdName + '.pcd'):
            i = 1
            while os.path.exists(folder + pcdName + str(i) + '.pcd'):
                i += 1
            pcdName = pcdName + str(i)
        pcdName = pcdName + '.pcd'

        #write to disk
        success = o3d.io.write_point_cloud(folder + pcdName, pcd, write_ascii=True) #ascii needs more space but is readable
        if not success:
            return jsonify(Message='Failed to write reference scan pcd')
        
        #commit to database
        userCompany = Company.query.get(current_user.company_id)
        newmodel = Model(filename=pcdName, company=userCompany)
        db.session.add(newmodel)
        db.session.commit()
        return jsonify({"filename": newmodel.filename})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/sendModelToWorkhorse/<app>", methods=['POST'])
@login_required
def sendModelToWorkhorse(app, filename_ = ""):
    try:
        if filename_ == "":
            filename_ = request.get_data(as_text=True)
        modelpath = os.path.join(current_app.root_path, upload_folder_path, filename_)
        JsonDict = {}
        fileFormat = modelpath.lower().rsplit('.', 1)[-1]
        if fileFormat in meshFormats:
            mesh = o3d.io.read_triangle_mesh(modelpath)
            JsonDict = {"vertices": np.asarray(mesh.vertices).tolist(), "triangles": np.asarray(mesh.triangles).tolist()}
        elif fileFormat in specialFormats:
            modelpath = modelpath + ".obj"
            mesh = o3d.io.read_triangle_mesh(modelpath)
            JsonDict = {"vertices": np.asarray(mesh.vertices).tolist(), "triangles": np.asarray(mesh.triangles).tolist()}
        elif fileFormat in pcdFormats:
            pcd = o3d.io.read_point_cloud(modelpath)
            JsonDict = {"points": np.asarray(pcd.points).tolist(), "colors": np.asarray(pcd.colors).tolist()}
        elif fileFormat in sharedFormats:
            isMesh = True
            model = o3d.io.read_triangle_mesh(modelpath)
            if model.is_empty():
                isMesh = False
                model = o3d.io.read_point_cloud(modelpath)
            if isMesh:
                JsonDict = {"vertices": np.asarray(model.vertices).tolist(), "triangles": np.asarray(model.triangles).tolist()}
            else:
                JsonDict = {"points": np.asarray(model.points).tolist(), "colors": np.asarray(model.colors).tolist()}
        else:
            print("Unknown file format in sendModelToWorkhorse!")
        res = requests.post('http://{}/receive3DModel/{}'.format(current_user.used_system.credentials, app), json=JsonDict)
        if res.ok:
            return jsonify(Message='OK')
        else:
            return jsonify(Message='Failed to send model to workhorse')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/addNewWorkhorseToDB", methods=['POST'])
def addNewWorkhorseToDB():
    try:
        # todo delete unassigned workhorses somewhere, here is not optimal
        # whs_invalid = Workhorse.query.filter_by(currentsystem_id = None)
        # if (whs_invalid.first() is not None):
        #     for wh in whs_invalid:
        #         db.session.delete(wh)
        #     db.session.commit()

        workhorseAdress = request.get_data(as_text=True)
        exists = db.session.query(db.exists().where(Workhorse.credentials == workhorseAdress)).scalar()
        if exists == False:
            newWorkhorse = Workhorse(credentials=workhorseAdress, usedInSystem=False)
            db.session.add(newWorkhorse)
            db.session.commit()   
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route('/SetCurrentSystem/<sysname>')
@login_required
def SetCurrentSystem(sysname):
    try:
        system = System.query.filter_by(name= sysname).first()
        if not system:
            return jsonify(Message=False)
        if system.name == current_user.used_system:
            return jsonify({'SlaveAdress': system.credentials})
        current_user.used_system = system
        db.session.commit()
        return jsonify({'SlaveAdress': system.credentials})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route('/getCurrentSystemData')
@login_required
def getCurrentSystemData():
    try:
        if current_user.used_system == None:
            return jsonify(Message="No system set")
        system = current_user.used_system
        serialNumbers = []
        CameraSettings = []
        for cam in system.cameras:
            serialNumbers.append(cam.serialnumber)
            CameraSettings.append(cam.used_settings.as_dict())
        if current_user.used_system.used_volumesettings.referenceModel != None:
            referenceModelName = current_user.used_system.used_volumesettings.referenceModel.filename
        else :
            referenceModelName = ''

        VolumeSettings = system.used_volumesettings.as_dict()
        QualitySettings = system.used_qualitysettings.as_dict()
        return jsonify({'Adress': system.credentials, 'serials': serialNumbers, 'modelName': referenceModelName,
                        'CameraSettings': CameraSettings, 'VolumeSettings': VolumeSettings, 'QualitySettings': QualitySettings})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route('/getDBSystems')
@login_required
def getDBSystems():
    try:
        out = []
        for sys in System.query.all():
            out.append(sys.as_dict())
        return jsonify(content = out)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

#adds a new system to the database
@system.route('/addNewSystemToDB', methods=['POST'])
@login_required
def addNewSystemToDB():
    try:   
        systemData= request.json
        company = Company.query.get(current_user.company_id)
        workhorse= Workhorse.query.filter_by(credentials= systemData['credentials']).first()
        newSystem = System(credentials=systemData['credentials'], name= systemData['name'], company_id=company.id, used_workhorse=workhorse)
        db.session.add(newSystem)
        for cam in systemData['cams']:
            camtype = cam.split(" : ")[0]
            serialnumber = cam.split(" : ")[1]
            newCam = Camera(serialnumber=serialnumber, system=newSystem, type = camtype)
            db.session.add(newCam)
        db.session.commit()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route('/getSystemCameraNames/<sysname>')
@login_required
def getSystemCameraNames(sysname):
    try:
        system= System.query.filter_by(name=sysname).first()
        if not system:
            return jsonify(Message=False)
        out = []
        for cam in system.cameras:
            out.append(cam.type + " : "  + cam.serialnumber)
        return jsonify(content = out)
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route('/getCamsOfCurrentSystem')
@login_required
def getCamsOfCurrentSystem():
    try:
        system= current_user.used_system
        serialNumbers = []
        for cam in system.cameras:
            serialNumbers.append(cam.serialnumber)
        return jsonify({'serialnumbers': serialNumbers})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route("/DeleteSystem", methods=['POST'])
@login_required
def DeleteSystem():
    try:
        systemName = request.data.decode("utf-8")
        system = System.query.filter_by(name=systemName).first()
        if not system:
            return jsonify(Message='System not found in db')
        serialnumbers = []
        for camera in system.cameras:
            serialnumbers.append(camera.serialnumber)
        try:
            res = requests.post('http://{}/isActive'.format(system.credentials), json={'serialnumbers':serialnumbers})
            if res.json()['Message'] != False: #todo sucks
                deactivateCurrentSystem()
        except requests.exceptions.ConnectionError:
            # In the event of a network problem (e.g. DNS failure, refused connection, etc), Requests will raise a ConnectionError exception.
            pass
        workhorse= Workhorse.query.filter_by(credentials= system.credentials).first()
        workhorse.usedInSystem = False
        db.session.delete(system) #also deletes cameras and system settings (volume and quality)
        db.session.commit()
        return jsonify({'credentials': system.credentials})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route("/DeleteModel", methods=['POST'])
@login_required
def DeleteModel():
    try:
        fileName = request.json['filename']
        model = Model.query.filter_by(filename=fileName).first()
        db.session.delete(model)
        db.session.commit()
        modelPath = os.path.join(current_app.root_path, upload_folder_path, fileName)
        fileFormat = modelPath.lower().rsplit('.', 1)[-1]
        if fileFormat in specialFormats:
            modelPath = modelPath + ".obj"
        if os.path.exists(modelPath):
            os.remove(modelPath)
        else:
            print(f"Cannot delete non existent model file at: {modelPath}")
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route("/getSystemCredentials", methods=['POST'])
@login_required
def getSystemCredentials():
    try:
        systemName = request.data.decode("utf-8")
        system = System.query.filter_by(name=systemName).first()
        return jsonify({'credentials': system.credentials})
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route("/changeCameraSettings", methods=['POST'])
@login_required
def changeCameraSettings():
    try:
        json = request.json
        cameraObj = False
        dbCameras = current_user.used_system.cameras
        for cam in dbCameras:
            if cam.serialnumber == json['serialnumber']:
                cameraObj=cam
                break
        camera = []

        if not cameraObj:
            # print("using last camera in list")
            camera = current_user.used_system.cameras[-1].used_settings
        else:
            camera = cameraObj.used_settings
        if 'ColorThresholdMin' in json:
            camera.ColorThresholdMin = json['ColorThresholdMin']
        if 'ColorThresholdMax' in json:
            camera.ColorThresholdMax = json['ColorThresholdMax']
        if 'crop_left' in json:
            camera.crop_left = json['crop_left']
        if 'crop_right' in json:
            camera.crop_right = json['crop_right']
        if 'crop_top' in json:
            camera.crop_top = json['crop_top']
        if 'crop_bottom' in json:
            camera.crop_bottom = json['crop_bottom']
        if 'crop_depth' in json:
            camera.crop_depth = json['crop_depth']
        if 'crop_depth_inv' in json:
            camera.crop_depth_inv = json['crop_depth_inv']
        if 'crop_ground_height' in json:
            camera.crop_ground_height = json['crop_ground_height']
        if 'crop_ground_height_fine' in json:
            camera.crop_ground_height_fine = json['crop_ground_height_fine']
        if 'dmax' in json:
            camera.dmax = json['dmax']
        if 'gravityvector' in json:
            camera.gravityvector = str(json['gravityvector']).translate({ord(c): None for c in '[] '})
        if 'extrinsic' in json:
            camera.extrinsic = str(json['extrinsic']).translate({ord(c): None for c in '[] '})
        
        db.session.commit()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route("/changeVolumeSettings", methods=['POST'])
@login_required
def changeVolumeSettings():
    try:
        json = request.json
        volume = current_user.used_system.used_volumesettings
        if 'xposition' in json:
            volume.x_offset = json['xposition']
        if 'yposition' in json:
            volume.y_offset = json['yposition']
        if 'zposition' in json:
            volume.z_offset = json['zposition']
        if 'xrotation' in json:
            volume.alpha = json['xrotation']
        if 'yrotation' in json:
            volume.beta = json['yrotation']
        if 'zrotation' in json:
            volume.gamma = json['zrotation']
        if 'TransMatrix' in json:
            volume.SiloToFirstCamTransform = str(json['TransMatrix']).translate({ord(c): None for c in '[] '})
        if 'TranslationOffset' in json:
            volume.TranslationOffset = str(json['TranslationOffset']).translate({ord(c): None for c in '[] '})
        if 'VolumeNoise' in json:
            volume.VolumeNoise = json['VolumeNoise']
        if 'DensityFactor' in json:
            volume.DensityFactor = json['DensityFactor']
        if 'BeltSpeed' in json:
            volume.BeltSpeed = json['BeltSpeed']
        if 'DirectionVector' in json:
            volume.DirectionVector = str(json['DirectionVector']).translate({ord(c): None for c in '[] '})
        if 'BasePlanePoint' in json:
            volume.BasePlanePoint = str(json['BasePlanePoint']).translate({ord(c): None for c in '[] '})
        if 'MaxProductHeight' in json:
            volume.MaxProductHeight = json['MaxProductHeight']
        if 'ProductPresenceThreshold' in json:
            volume.ProductPresenceThreshold = json['ProductPresenceThreshold']
        if 'UseBeltSpeedEndpoint' in json:
            volume.UseBeltSpeedEndpoint = json['UseBeltSpeedEndpoint']
        if 'BeltSpeedEndpointAddress' in json:
            volume.BeltSpeedEndpointAddress = json['BeltSpeedEndpointAddress']
        if 'BeltSpeedEndpointPayload' in json:
            volume.BeltSpeedEndpointPayload = json['BeltSpeedEndpointPayload']
        if 'BeltSpeedEndpointDistance' in json:
            volume.BeltSpeedEndpointDistance = json['BeltSpeedEndpointDistance']
        if 'BeltSpeedEndpointUsername' in json:
            volume.BeltSpeedEndpointUsername = json['BeltSpeedEndpointUsername']
        if 'BeltSpeedEndpointPassword' in json:
            volume.BeltSpeedEndpointPassword = json['BeltSpeedEndpointPassword']
        if 'filename' in json:
            model = Model.query.filter_by(filename=json["filename"]).first()
            if model:
                volume.referenceModel = model
            else:
                print('Model not found in db')
        db.session.commit()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route("/changeQualitySettings", methods=['POST'])
@login_required
def changeQualitySettings():
    try:
        json = request.json
        quality = current_user.used_system.used_qualitysettings

        if 'Pattern_h' in json:
            quality.Pattern_h = json['Pattern_h']
        if 'Pattern_w' in json:
            quality.Pattern_w = json['Pattern_w']
        if 'SquareLength' in json:
            quality.SquareLength = json['SquareLength']
        if 'RefineAlignWithCAD' in json:
            quality.RefineAlignWithCAD = json['RefineAlignWithCAD']
        if 'UseVoxelGrid' in json:
            quality.UseVoxelGrid = json['UseVoxelGrid']
        if 'filename' in json:
            model = Model.query.filter_by(filename=json["filename"]).first()
            if model:
                quality.referenceModel = model
            else:
                print('Model not found in db')
        db.session.commit()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/deleteUser", methods=['POST'])
def deleteUser():
    try:
        userId = int(request.data.decode("utf-8"))
        user = User.query.get(userId)
        if user.id != current_user.id:
            db.session.delete(user)
            db.session.commit()
            return jsonify(Message='OK')
        else:
            return jsonify(Message="Can't delete current user")
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/registerNewUser", methods=['POST'])
def registerNewUser():
    try:
        data= request.json
        current_company= current_user.company
        newuser= User(email=data['newemail'],name=data['newfirstname'],password=data['newpassword'],role=data['role'],company_id= current_company.id)
        db.session.add(newuser)
        db.session.commit()
        flash("Successfully added new User.", category="successnewuser")
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)

@system.route("/changeUserData", methods=['POST'])
def changeUserData():
    try:
        data= request.json
        user= User.query.get(int(data['id']))
        user.name = data['newname']
        user.email = data['newmail']
        db.session.commit()
        return jsonify(Message='OK')
    except Exception:
        logger.error("Exception:" + traceback.format_exc())
        return Response(status=420)


@system.route('/activateCurrentSystem')
@login_required
def activateCurrentSystem():
    sys = current_user.used_system
    if sys:
        return activateSystem(sys.name)
    else:
        return jsonify(Message="No current system set")


@system.route('/activateSystem/<systemName>')
def activateSystem(systemName):
    #check if systemName is in Database
    system = System.query.filter_by(name=systemName).first()
    if not system:
        return jsonify(Message='System not found in database')
    
    serialnumbers = []
    for camera in system.cameras:
        serialnumbers.append(camera.serialnumber)
    #check if the system workhorse is available
    try:
        res = requests.post("http://" + system.credentials + "/isActive", json={'serialnumbers':serialnumbers})
        if not res.ok:
            return jsonify(Message='Workhorse ' + system.credentials + ' is unreachable (response not ok)')
        if res.json()["Message"] == "busy" or res.json()["Message"] == True:
            return jsonify(Message='Workhorse ' + system.credentials + ' is already busy')
    except requests.exceptions.ConnectionError:
        # In the event of a network problem (e.g. DNS failure, refused connection, etc), Requests will raise a ConnectionError exception.
        return jsonify(Message='Workhorse ' + system.credentials + ' is unreachable')
    #send all db data to the system workhorse and let the system activate (call workhorse.activateSystem)
    cameras = []
    for cam in system.cameras:
        cameras.append({"serialnumber": cam.serialnumber, "settings": cam.used_settings.as_dict(), "type": cam.type})
    sysinfo = {
        'system': system.as_dict(),
        'cameras': cameras, 
        'VolumeSettings': system.used_volumesettings.as_dict(), 
        'QualitySettings': system.used_qualitysettings.as_dict()
    }
    res = requests.post("http://" + system.credentials + "/activateSystemWorkhorse", json=sysinfo)
    if res.json()['Message'] != 'OK':
        return jsonify(res.json())

    if system.used_qualitysettings.referenceModel != None:
        filename = system.used_qualitysettings.referenceModel.filename
        if filename != "": 
            sendModelToWorkhorse("Quality", filename)
    if system.used_volumesettings.referenceModel != None:
        filename = system.used_volumesettings.referenceModel.filename
        if filename != "": 
            sendModelToWorkhorse("Volume", filename)

    return jsonify(Message = "OK")



@system.route('/deactivateCurrentSystem')
@login_required
def deactivateCurrentSystem():
    system = current_user.used_system
    if not system:
        return jsonify(Message='Current user has no system')
    try:
        res = requests.get("http://" + system.credentials + "/clearCameraList")
        if not res.ok:
            print('Request clearCameraList failed, status code ' + res.status_code)
        res = requests.get("http://" + system.credentials + "/clearModels")
        if not res.ok:
            print('Request clearModels failed, status code ' + res.status_code)
    except requests.exceptions.ConnectionError:
        # In the event of a network problem (e.g. DNS failure, refused connection, etc), Requests will raise a ConnectionError exception.
        return jsonify(Message='Failed to deactivate system as workhorse is unreachable')
    db.session.commit()
    return jsonify(Message='OK')


@system.route('/getCurrentSystem')
@login_required
def getCurrentSystem():
    system = current_user.used_system
    if not system:
        return jsonify(Message = False)
    return jsonify(content = system.as_dict())
