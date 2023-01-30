from fileinput import filename
from sqlalchemy import false
from sqlalchemy.orm import relationship, backref
from flask_login import UserMixin
from flask_sqlalchemy import SQLAlchemy
from werkzeug.security import generate_password_hash

db = SQLAlchemy()

#note: currently all units of db class fields match the GUI (not necessarily SI-units like in cpp classes)

class Company(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(150))
    adress = db.Column(db.String(200))
    town = db.Column(db.String(200)) #add to address
    employees = db.relationship('User', back_populates='company')
    models = db.relationship('Model', back_populates='company')
    systems = db.relationship('System', back_populates='company')
    def __repr__(self) :
        return self.name

class User(db.Model, UserMixin):
    id = db.Column(db.Integer, primary_key=True)
    email = db.Column(db.String(150), unique=True)
    password = db.Column(db.String(500))
    name = db.Column(db.String(150))
    role = db.Column(db.String(20))
    licenses = db.Column(db.String(150)) #todo extra class
    license_exp_date = db.Column(db.DateTime)
    company_id = db.Column(db.Integer, db.ForeignKey('company.id'), nullable=False)
    company = db.relationship('Company', back_populates='employees')
    used_system = db.relationship('System', back_populates='currentuser', uselist=False) ##
    def __init__(self, **kwargs):
        super().__init__(**kwargs) # base model class constructor which takes and sets all arguments given
        if 'password' in kwargs:
            self.password = generate_password_hash(kwargs.get('password'), method='sha256')


class Model(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    filename = db.Column(db.String(50), nullable=False)
    company_id = db.Column(db.Integer, db.ForeignKey('company.id'), nullable=False)
    company = db.relationship('Company', back_populates='models')
    volumesettings= db.relationship('VolumeSettings', back_populates='referenceModel')
    qualitysettings= db.relationship('QualitySettings', back_populates='referenceModel')
    def __init__(self, **kwargs):
        self.filename = ""
        super().__init__(**kwargs) # base model class constructor which takes and sets all arguments given

    
class System(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    credentials = db.Column(db.String(30), nullable=False, unique=False) # todo this is same as workhorse credentials, think through system creation
    name= db.Column(db.String(50))
    company_id = db.Column(db.Integer, db.ForeignKey('company.id'), nullable=False)
    company = db.relationship('Company', back_populates='systems')
    currentuser_id = db.Column(db.Integer, db.ForeignKey('user.id'), nullable=True)
    currentuser= db.relationship('User', back_populates="used_system")
    cameras = db.relationship('Camera', back_populates='system', cascade='all,delete,delete-orphan')
    used_volumesettings = db.relationship('VolumeSettings', back_populates='currentsystem', cascade='all,delete,delete-orphan', uselist= False) ##
    used_qualitysettings = db.relationship('QualitySettings', back_populates='currentsystem', cascade='all,delete,delete-orphan', uselist= False) ##
    used_workhorse = db.relationship('Workhorse', back_populates='currentsystem', uselist=False) ##
    def __init__(self, **kwargs):
        self.name = "Unnamed_System"
        self.used_volumesettings = VolumeSettings(name=self.name+'_VolumeSettings')
        self.used_qualitysettings =QualitySettings(name=self.name+'_QualitySettings')
        super().__init__(**kwargs) # base model class constructor which takes and sets all arguments given
    def as_dict(self): #todo add relational stuff
       return {c.name: getattr(self, c.name) for c in self.__table__.columns}




class VolumeSettings(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(20))
    referenceModel_id = db.Column(db.Integer, db.ForeignKey('model.id'), nullable=True)
    referenceModel = db.relationship('Model', back_populates='volumesettings')
    currentsystem_id = db.Column(db.Integer, db.ForeignKey('system.id'), nullable=True)
    currentsystem = db.relationship('System', back_populates="used_volumesettings")
    #calibration, offset for model
    x_offset = db.Column(db.Float)
    y_offset = db.Column(db.Float)
    z_offset = db.Column(db.Float)
    alpha = db.Column(db.Float)
    beta = db.Column(db.Float)
    gamma = db.Column(db.Float)
    #transmat mat4x4
    SiloToFirstCamTransform= db.Column(db.String(400))
    #translationoffset vec3
    TranslationOffset= db.Column(db.String(100))
    #extra
    VolumeNoise=db.Column(db.Float)
    DensityFactor=db.Column(db.Float)
    BeltSpeed=db.Column(db.Float)
    DirectionVector=db.Column(db.String(100))
    BasePlanePoint=db.Column(db.String(100))
    MaxProductHeight=db.Column(db.Float)
    ProductPresenceThreshold=db.Column(db.Float)
    #beltspeed endpoint
    UseBeltSpeedEndpoint=db.Column(db.Boolean)
    BeltSpeedEndpointAddress=db.Column(db.String(100))
    BeltSpeedEndpointPayload=db.Column(db.String(250))
    BeltSpeedEndpointDistance=db.Column(db.Float)
    BeltSpeedEndpointUsername=db.Column(db.String(150))
    BeltSpeedEndpointPassword=db.Column(db.String(150))
    def __init__(self, **kwargs):
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        self.alpha = 0
        self.beta = 0
        self.gamma = 0
        self.SiloToFirstCamTransform = '1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1'
        self.TranslationOffset = '0,0,0'
        self.VolumeNoise = -7.0
        self.DensityFactor = 1.0
        self.BeltSpeed = 0.0
        self.DirectionVector = '0,0,0'
        self.BasePlanePoint = '0,0,0'
        self.MaxProductHeight = 100
        self.ProductPresenceThreshold = 1
        self.UseBeltSpeedEndpoint = False
        self.BeltSpeedEndpointAddress = "172.16.10.40:39320/iotgateway/read"
        self.BeltSpeedEndpointPayload = "TZ2_UDC001.UDC001.ServerInterfaces.PLC_1.OPC.Totaliser_Count1"
        self.BeltSpeedEndpointDistance = 0.0
        self.BeltSpeedEndpointUsername = "pandia"
        self.BeltSpeedEndpointPassword = "mkxr3HpzDUvQHDA"
        super().__init__(**kwargs) # base model class constructor which takes and sets all arguments given
    def as_dict(self):
        return {c.name: getattr(self, c.name) for c in self.__table__.columns}


class QualitySettings(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    name = db.Column(db.String(20))
    referenceModel_id = db.Column(db.Integer, db.ForeignKey('model.id'), nullable=True)
    referenceModel = db.relationship('Model', back_populates='qualitysettings')
    currentsystem_id = db.Column(db.Integer, db.ForeignKey('system.id'), nullable=True)
    currentsystem = db.relationship('System', back_populates="used_qualitysettings")
    #settings

    Pattern_h = db.Column(db.Integer)
    Pattern_w = db.Column(db.Integer)
    SquareLength = db.Column(db.Float)
    RefineAlignWithCAD = db.Column(db.Boolean)
    UseVoxelGrid = db.Column(db.Boolean)
    def __init__(self, **kwargs):

        self.Pattern_h = 6
        self.Pattern_w = 9
        self.SquareLength = 48.5
        self.RefineAlignWithCAD = True
        self.UseVoxelGrid = False
        super().__init__(**kwargs)
    def as_dict(self):
        return {c.name: getattr(self, c.name) for c in self.__table__.columns}

#one hardware camera can be used in mutliple systems and will be saved
#redundantly 
class Camera(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    serialnumber= db.Column(db.String(25))
    system_id = db.Column(db.Integer, db.ForeignKey('system.id'), nullable= False)
    system = db.relationship('System', back_populates='cameras')
    used_settings = db.relationship('CameraSettings', back_populates='camera', cascade='all,delete,delete-orphan', uselist=False)
    type = db.Column(db.String(25))
    def __init__(self, **kwargs):
        self.used_settings = CameraSettings()
        self.type = "undefined"
        super().__init__(**kwargs) # base model class constructor which takes and sets all arguments given

class CameraSettings(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    camera = db.relationship('Camera', back_populates='used_settings')
    camera_id = db.Column(db.Integer, db.ForeignKey('camera.id'), nullable=True)
    #fov
    crop_left = db.Column(db.Float)
    crop_right = db.Column(db.Float)
    crop_top = db.Column(db.Float)
    crop_bottom = db.Column(db.Float)
    crop_depth = db.Column(db.Float)
    crop_depth_inv = db.Column(db.Float)
    crop_ground_height = db.Column(db.Float)
    crop_ground_height_fine = db.Column(db.Float)
    #fovtreshholds
    dmax = db.Column(db.Float)
    #distance thresholds
    ColorThresholdMin = db.Column(db.Float)
    ColorThresholdMax = db.Column(db.Float)
    #pose
    gravityvector = db.Column(db.String(100)) #vec3d
    extrinsic = db.Column(db.String(400))  # mat4d
    def __init__(self, **kwargs):
        self.crop_left = 0
        self.crop_right = 0
        self.crop_top = 0
        self.crop_bottom = 0
        self.crop_depth = 0
        self.crop_depth_inv = 0
        self.crop_ground_height = 1e6
        self.crop_ground_height_fine = 0
        self.dmax = 5
        self.ColorThresholdMin = 5
        self.ColorThresholdMax = 5
        self.gravityvector = '0,0,-1'
        self.extrinsic = '1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1'
        super().__init__(**kwargs) # base model class constructor which takes and sets all arguments given
    def as_dict(self):
        return {c.name: getattr(self, c.name) for c in self.__table__.columns}

class Workhorse(db.Model):
    id = db.Column(db.Integer, primary_key=True)
    currentsystem_id = db.Column(db.Integer, db.ForeignKey('system.id'), nullable=True)
    currentsystem= db.relationship('System', back_populates="used_workhorse")
    usedInSystem= db.Column(db.Boolean)
    credentials = db.Column(db.String(30), nullable=False, unique=True)
    def as_dict(self):
       return {c.name: getattr(self, c.name) for c in self.__table__.columns}


# class Licenses(db.Model):
#     id = db.Column(db.Integer, primary_key=True)
#     VolumeScanner = db.Column(db.Boolean, default=False)
#     QualityScanner = db.Column(db.Boolean, default=False)
#     OCMI = db.Column(db.Boolean, default=False)