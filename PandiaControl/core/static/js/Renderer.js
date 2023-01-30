
import * as THREE from '../3rdpartylibs/three.js-master/build/three.module.js'
import { OBJLoader } from '../3rdpartylibs/three.js-master/examples/jsm/loaders/OBJLoader.js'
import { PCDLoader } from '../3rdpartylibs/three.js-master/examples/jsm/loaders/PCDLoader.js'
import { STLLoader } from '../3rdpartylibs/three.js-master/examples/jsm/loaders/STLLoader.js'
import { GLTFLoader } from '../3rdpartylibs/three.js-master/examples/jsm/loaders/GLTFLoader.js'
import { XYZLoader } from '../3rdpartylibs/three.js-master/examples/jsm/loaders/XYZLoader.js'
import { PLYLoader } from '../3rdpartylibs/three.js-master/examples/jsm/loaders/PLYLoader.js'
import { TrackballControls } from '../3rdpartylibs/three.js-master/examples/jsm/controls/TrackballControls.js'
// import { OrbitControls } from '../3rdpartylibs/three.js-master/examples/jsm/controls/OrbitControls.js'
import {enableShowButtons} from './QualityControl/QualityControl.js'
import * as utils from './utils.js'

//note: we currently flip all models in three js (meshes with getFlipmatrix, pcds with scale)
//############# general reference models #################
export let CurrentReferenceModel = new THREE.Object3D;
export function setCurrentReferenceModel(model) {
    CurrentReferenceModel = model;
}

export let FOVPcd; //cropped pcd for fov settings
export function setFOVPcd(model) {
    FOVPcd = model;
    FOVPcd.scale.set(1,-1,-1)
}

//##################### Conveyor Belt #####################
export let FirstBeltPcd; //cropped pcd for fov settings
export function setFirstBeltPcd(model) {
    FirstBeltPcd = model;
    FirstBeltPcd.scale.set(1,-1,-1)
}

export let SecondBeltPcd; //cropped pcd for fov settings
export function setSecondBeltPcd(model) {
    SecondBeltPcd = model;
    SecondBeltPcd.scale.set(1,-1,-1)

}

export let DirectionArrow;
export function setDirectionArrow(model) {
    DirectionArrow = model;
    // DirectionArrow.scale.set(1,-1,-1)
}

//############# quality control models #################
export let PCDArray = [] //pcds of scan, are saved seperately for different colorings, and point sizes
export let PCDColorsArray = [] //redundant color save 
export let PCDGradientArray = [] //colors that indicate errors

export function pushToPCDArray(model) {
    model.scale.set(1,-1,-1)
    PCDArray.push(model)
}
export function pushToPCDColorsArray(colors) {
    PCDColorsArray.push(colors)
}
export function pushToPCDGradientArray(colors) {
    PCDGradientArray.push(colors)
}

export function addPCDsToScene() {
    for (var i = 0; i < PCDArray.length; i++){
        Scene.add(PCDArray[i]);
        PCDArray[i].geometry.setAttribute('color', PCDColorsArray[i]);
    }
}
export function addPCDsToSceneGradient() {
    for (var i = 0; i < PCDArray.length; i++){
        Scene.add(PCDArray[i]);
        PCDArray[i].geometry.setAttribute('color', PCDGradientArray[i]);
    }
}
export function clearPCDsAndColors() {
    PCDArray.length = 0
    PCDColorsArray.length = 0
    PCDGradientArray.length = 0
}


export let MissingPoints;
export function setMissingPoints(model) {
    MissingPoints = model;
    MissingPoints.scale.set(1,-1,-1)
}
export let VisibilityPCD;
export function setVisibilityPCD(model) {
    VisibilityPCD = model;
    VisibilityPCD.scale.set(1,-1,-1)
}
export let CalibrationPCD;
export function setCalibrationPCD(model) {
    CalibrationPCD = model;
    CalibrationPCD.scale.set(1,-1,-1)
}


//############# basic renderer #################
export let Scene = new THREE.Scene();
export let CameraModel = new THREE.Object3D;
export let Controls;

//@ts-ignore
if (J_SideBarMode == 'active' && J_DisplayMode == '3drenderer' || J_SideBarMode == 'setup') 
{
    let BoundingRect = document.getElementById('id_RenderHull').getBoundingClientRect(); //gets the size of the render window
    const HtmlCanvas = document.getElementById("id_RenderCanvas");
    //@ts-ignore
    HtmlCanvas.width = BoundingRect.width;
    //@ts-ignore
    HtmlCanvas.height = BoundingRect.height;

    const Renderer = new THREE.WebGLRenderer({ canvas: HtmlCanvas, antialias: true, alpha: true });
    // Renderer.setClearColor(0xf1f1f1, 1)
    Renderer.setSize(BoundingRect.width, BoundingRect.height);
    //@ts-ignore
    Renderer.shadowMap.enabled = true;
    //@ts-ignore
    Renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    //light
    const LightColor = new THREE.Color(1, 1, 1);
    const PointLight = new THREE.PointLight(LightColor, 0.5)
    const AmbientLight = new THREE.AmbientLight(LightColor, 0.5);
    Scene.add(AmbientLight);

    //camera
    const Camera = new THREE.PerspectiveCamera(45, BoundingRect.width / BoundingRect.height, 10, 100000);
    Scene.add(Camera);
    Camera.add(PointLight);
    //@ts-ignore
    Camera.position.set(0, 10, 1000);

    //camera controls
    Controls = new TrackballControls(Camera, Renderer.domElement);
    Controls.staticMoving = false;
    Controls.dynamicDampingFactor = 0.8;
    Controls.rotateSpeed = 4.0;
    Controls.zoomSpeed = 3.0;
    Controls.panSpeed = 0.6;

    // Old camera Controls - orbit cam
    // const controls = new OrbitControls(Camera, Renderer.domElement);
    // controls.update();
    // controls.saveState();

    //###### Load CameraModels ###########
    //@ts-ignore
    if (J_SideBarMode == 'setup') {
        //todo different camera models for each type
        const objloader = new OBJLoader();
        const camMaterial = new THREE.MeshPhongMaterial({ color: 0x404040 })
        objloader.load('/static/CameraModels/Ensenso_S10.obj', function (model) {
            model.scale.set(20, 20, 20)
            model.traverse(function (child) {
                if (child.isMesh)
                    child.material = camMaterial;
            });
            CameraModel = model;
        })
    }

    //########KeyPresse Events##########
    let wireOnOff = false;
    let setTransparent = false;
    document.addEventListener("keydown", KeyPressed, false);
    function KeyPressed(event) 
    {
        let keyCode = event.which;
        //Wireframe
        if (keyCode == 87) //W
        {
            wireOnOff = !wireOnOff
            CurrentReferenceModel.traverse(function (child) {
                if (child.isMesh)
                    child.material.wireframe = wireOnOff;
            });
        }
        //Reset Camera
        if (keyCode == 82) //R
        {
            Controls.reset();
        }
        //Toggle transparancy T
        if (keyCode == 84) {
            setTransparent = !setTransparent
            CurrentReferenceModel.traverse(function (child) {
                if (child.isMesh && setTransparent)
                    child.material.opacity = 0.5
                else if (child.isMesh && !setTransparent)
                    child.material.opacity = 1
            })
        }
        //Change Point size
        if (keyCode == 107 || keyCode == 171) //+
        {
            changePointSize(FOVPcd, 1.25);
            changePointSize(MissingPoints, 1.25);
            changePointSize(FirstBeltPcd, 1.25);
            changePointSize(SecondBeltPcd, 1.25);
            changePointSize(VisibilityPCD, 1.25);
            changePointSize(CalibrationPCD, 1.25);
            changePointSize(CurrentReferenceModel, 1.25);
            for (var i = 0; i < PCDArray.length; i++){
                changePointSize(PCDArray[i], 1.25)
            }
        }
        if (keyCode == 109 || keyCode == 173) //-
        {
            changePointSize(FOVPcd, 0.75);
            changePointSize(MissingPoints, 0.75);
            changePointSize(FirstBeltPcd, 0.75);
            changePointSize(SecondBeltPcd, 0.75);
            changePointSize(VisibilityPCD, 0.75);
            changePointSize(CalibrationPCD, 0.75);
            changePointSize(CurrentReferenceModel, 0.75);
            for (var i = 0; i < PCDArray.length; i++){
                changePointSize(PCDArray[i], 0.75)
            }
        }
        if (keyCode == 85) //U
        {
            enableShowButtons();
            try { document.getElementById("id_AddScan").classList.remove('disabled') } catch { };
            try { document.getElementById("id_RevertLastScan").classList.remove('disabled') } catch { };
        }
    }

    // resizing of renderer event
    window.addEventListener('resize', onWindowResize, false)
    function onWindowResize() {
        //@ts-ignore
        let BoundingRect = document.getElementById('id_RenderHull').getBoundingClientRect()
        Camera.aspect = BoundingRect.width / BoundingRect.height
        Camera.updateProjectionMatrix()
        Renderer.setSize(BoundingRect.width, BoundingRect.height)
    }

    // To render the scene 
    const updateframe = function () {
        requestAnimationFrame(updateframe);
        Controls.update();
        Renderer.render(Scene, Camera);
    };
    updateframe();
}


function changePointSize(pcd, factor)
{
    try {
        pcd.traverse((child) => {
            if (child instanceof THREE.Points) {
                child.material.size *= factor;
            }
            else
                return;
        });
    } catch (error) { return }
}

export async function LoadNewModelinRenderer(filename_in) {
    let filename
    if (filename_in === undefined) {
        let select = document.getElementById('id_3DModelList');
        //@ts-ignore
        try { filename = select.options[select.selectedIndex].text; }
        catch (error) { return }
    }
    else {
        filename = filename_in
    }

    let pathAndFileName = '/static/3dModels/' + filename;
    let model;
    let extension = pathAndFileName.split('.').pop().toLowerCase();
    
    const meshMaterial = new THREE.MeshPhongMaterial({ color: new THREE.Color("rgb(150,150,150)"), transparent: true, opacity: 0.5 });
    meshMaterial.side = THREE.DoubleSide
    const pcdMaterial = new THREE.PointsMaterial({color: 0x222222, opacity: 0.3, size: G_PointSize, transparent: true})
    
    // supported o3d formats
    // meshFormats = {'stl', 'obj', 'off', 'gltf'}
    // pcdFormats = {'xyz', 'xyzn', 'xyzrgb', 'pts', 'pcd'}
    // sharedFormats = {'ply'}
    let loader;
    let geometry; //note: some loaders simply return a geometry instead of a mesh or points
    switch (extension) {
        //shared formats
        case 'ply':
            loader = new PLYLoader();
            geometry = await loader.loadAsync(pathAndFileName)
            model = new THREE.Mesh(geometry);
            break;
        //mesh
        case 'stl':
            loader = new STLLoader();
            geometry = await loader.loadAsync(pathAndFileName)
            model = new THREE.Mesh(geometry);
            break;
        case 'obj':
            loader = new OBJLoader();
            model = await loader.loadAsync(pathAndFileName)
            break;
        case 'gltf':
            loader = new GLTFLoader();
            let gltf = await loader.loadAsync(pathAndFileName)
            gltf.scene.traverse(function(child){
                if (child.isMesh) {
                    model = child
                }
            })
            break;
        //pcd
        case 'pcd':
            loader = new PCDLoader();
            model = await loader.loadAsync(pathAndFileName)
            break;
        case 'xyz':
            //fall through
        case 'xyzrgb':
            loader = new XYZLoader();
            geometry = await loader.loadAsync(pathAndFileName)
            model = new THREE.Points(geometry);
            break;
        //special formats are saved as obj
        case 'stp':
            //fall trough
        case 'step':
            loader = new OBJLoader();
            pathAndFileName = pathAndFileName + ".obj";
            model = await loader.loadAsync(pathAndFileName)
            break;
        default:
            console.log("Data Type " + extension + " is not supported yet!");
            break;
    }

    model.traverse(function (child) {
        if (child.isMesh) {
            child.material = meshMaterial
        }
        else if (child.isPoints){
            child.material = pcdMaterial
        }
    })
    removeAllModels()
    setCurrentReferenceModel(model)
    CurrentReferenceModel.matrixAutoUpdate = false
    CurrentReferenceModel.matrix = utils.getFlipMatrix()
    CurrentReferenceModel.matrixWorldNeedsUpdate = true;
    Scene.add(CurrentReferenceModel)
}

export function removeAllModels() {
    for (var i = Scene.children.length - 1; i >= 0; i--)
    {
        let obj = Scene.children[i];
        if (obj instanceof THREE.Mesh || obj instanceof THREE.Points || obj instanceof THREE.ArrowHelper)
        {
            // if(obj.geometry) obj.geometry.dispose();
            // if(obj.material) { 
            //     //in case of map, bumpMap, normalMap, envMap ...
            //     Object.keys(obj.material).forEach(prop => {
            //       if(!obj.material[prop])
            //         return;
            //       if(obj.material[prop] !== null && typeof obj.material[prop].dispose === 'function')                                  
            //         obj.material[prop].dispose();                                                      
            //     })
            //     obj.material.dispose();
            // }
            Scene.remove(obj);
        }
    }
    Scene.remove(CameraModel)
    Scene.remove(CurrentReferenceModel)
    Scene.remove(FOVPcd)
    Scene.remove(MissingPoints)
    Scene.remove(VisibilityPCD)
    Scene.remove(CalibrationPCD)

    for (var i = 0; i < PCDArray.length; i++){
        Scene.remove(PCDArray[i]);
    }
}
