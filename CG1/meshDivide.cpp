
//! [fullsource]

#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include "OgreApplicationContext.h"
#include "OgreTrays.h"
#include <OgreCamera.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <ois/OIS.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <stb_image.h>

using namespace Ogre;
using namespace OgreBites;

std::mutex data_mutex;

struct point {
    Vector3 pos;
    Vector3 uv;
};
struct triangle {
    std::vector<point> node;
    Vector3 position;
};

struct KDNode {
    triangle* list;
    int num;
    AxisAlignedBox aabb;
    KDNode* left;
    KDNode* right;

    KDNode() {
        list = nullptr;
        num = 0;
        left = right = nullptr;
    }
};

enum Visibility
{
    NONE,
    PARTIAL,
    FULL
};

Visibility getVisibility(Camera* cam, const AxisAlignedBox& bound);

bool x_cmp(triangle p1, triangle p2) {
    return p1.position.x < p2.position.x;
}
bool y_cmp(triangle p1, triangle p2) {
    return p1.position.y < p2.position.y;
}
bool z_cmp(triangle p1, triangle p2) {
    return p1.position.z < p2.position.z;
}


bool getDataThread(Camera* cam, const KDNode* node, std::vector<std::pair<triangle*, int>>* data) {
    Visibility v = getVisibility(cam, node->aabb);
    if (v == FULL) {
        std::lock_guard<std::mutex> guard(data_mutex);
        data->push_back(std::pair<triangle*, int>(node->list, node->num));
        std::cout << "full\n";
        return true;
    }
    else if (v == PARTIAL) {
        std::cout << "partial\n";
        if (node->left)
            getDataThread(cam, node->left, data);
        if (node->right)
            getDataThread(cam, node->right, data);
        if (!node->left && !node->right) {
            triangle* tmpData = new triangle[node->num];
            int tmpNum = 0;
            for (int i = 0; i < node->num; ++i) {
                if (cam->isVisible(node->list[i].position))
                    tmpData[tmpNum++] = node->list[i];
            }
            triangle* tmpData1 = new triangle[tmpNum];
            for (int i = 0; i < tmpNum; ++i) {
                tmpData1[i] = tmpData[i];
            }
            delete[] tmpData;
            std::lock_guard<std::mutex> guard(data_mutex);
            data->push_back(std::pair<triangle*, int>(tmpData1, tmpNum));
        }
    }
    return false;
}

class KDTree {
public:
    KDNode* root;
    int maxDepth;
    int maxNumPerBox;
    int tmpCount;
    triangle* list;
    int num;
    KDTree() : root(nullptr), maxDepth(10), maxNumPerBox(100), tmpCount(0), list(nullptr), num(0) {};
    KDTree(triangle* list, int num, int maxD = 10, int maxN = 100) {
        maxDepth = maxD;
        maxNumPerBox = maxN;
        tmpCount = 0;
        this->num = num;
        this->list = new triangle[num];
        memcpy(this->list, list, num * sizeof(triangle));
        buildTree(this->list, num);
    }
    ~KDTree() {
        destroyTree(root);
        if (list)
            delete[] list;
    }
    bool buildTree(triangle* list, int num) {
        Vector3 minV = Vector3(FLT_MAX, FLT_MAX, FLT_MAX);
        Vector3 maxV = Vector3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        for (int i = 0; i < num; ++i) {
            minV.x = minV.x < list[i].position.x ? minV.x : list[i].position.x;
            minV.y = minV.y < list[i].position.y ? minV.y : list[i].position.y;
            minV.z = minV.z < list[i].position.z ? minV.z : list[i].position.z;
            maxV.x = maxV.x > list[i].position.x ? maxV.x : list[i].position.x;
            maxV.y = maxV.y > list[i].position.y ? maxV.y : list[i].position.y;
            maxV.z = maxV.z > list[i].position.z ? maxV.z : list[i].position.z;
        }
        AxisAlignedBox box(minV, maxV);
        return buildTreeImp(root, list, num, box, 0);
    }
    void destroyTree(KDNode* node) {
        if (!node)
            return;
        destroyTree(node->left);
        destroyTree(node->right);
        delete node;
    }
    std::pair<triangle*, int> getData(Camera* cam) {
        std::vector<std::pair<triangle*, int>> data;
        cam->setFOVy(Degree(60)); //三角形边缘有锯齿，稍微扩大了点视角
        //getDataImp(cam, root, data);
        std::thread th1(getDataThread, cam, root->left->left, &data);
        std::thread th2(getDataThread, cam, root->left->right, &data);
        std::thread th3(getDataThread, cam, root->right->left, &data);
        std::thread th4(getDataThread, cam, root->right->right, &data);
        th1.join();
        th2.join();
        th3.join();
        th4.join();
        std::vector<std::pair<triangle*, int>>::iterator it;
        int tmpNum = 0;
        for (it = data.begin(); it != data.end(); ++it) {
            tmpNum += it->second;
        }
        triangle* rtnData = new triangle[tmpNum];
        triangle* tmpHead = rtnData;
        for (it = data.begin(); it != data.end(); ++it) {
            memcpy(tmpHead, it->first, it->second * sizeof(triangle));
            tmpHead += it->second;
        }
        cam->setFOVy(Degree(45));
        //处理data造成的内存泄漏
        return std::pair<triangle*, int>(rtnData, tmpNum);
    }
private:
    bool buildTreeImp(KDNode*& node, triangle* list, int num, AxisAlignedBox& box, int depth) {
        node = new KDNode();
        node->list = list;
        node->num = num;
        node->aabb = box;
        tmpCount++;
        if (depth >= maxDepth || num < maxNumPerBox) {
            node->left = node->right = nullptr;
            return true;
        }
        int axis = int(3 * rand() / (RAND_MAX + 1));
        switch (axis)
        {
        case 0:
            std::sort(list, list + num, x_cmp);
            break;
        case 1:
            std::sort(list, list + num, y_cmp);
            break;
        case 2:
            std::sort(list, list + num, z_cmp);
            break;
        default:
            std::sort(list, list + num, x_cmp);
            break;
        }
        int tmpV = list[num / 2].position[axis];
        Vector3 minP = box.getMinimum();
        Vector3 maxP = box.getMaximum();
        Vector3 minMid = minP;
        Vector3 maxMid = maxP;
        minMid[axis] = tmpV - 0.1;
        maxMid[axis] = tmpV + 0.1;
        AxisAlignedBox lBox = AxisAlignedBox(minP, maxMid);
        AxisAlignedBox rBox = AxisAlignedBox(minMid, maxP);
        if (buildTreeImp(node->left, list, num / 2, lBox, depth + 1)
            && buildTreeImp(node->right, list + (num / 2), num - num / 2, rBox, depth + 1))
            return true;
    }
    bool getDataImp(Camera* cam, const KDNode* node, std::vector<std::pair<triangle*, int>>& data) {
        Visibility v = getVisibility(cam, node->aabb);
        if (v == FULL) {
            data.push_back(std::pair<triangle*, int>(node->list, node->num));
            std::cout << "full\n";
            return true;
        }
        else if (v == PARTIAL) {
            std::cout << "partial\n";
            if (node->left)
                getDataImp(cam, node->left, data);
            if (node->right)
                getDataImp(cam, node->right, data);
            if (!node->left && !node->right) {
                triangle* tmpData = new triangle[node->num];
                int tmpNum = 0;
                for (int i = 0; i < node->num; ++i) {
                    if (cam->isVisible(node->list[i].position))
                        tmpData[tmpNum++] = node->list[i];
                }
                triangle* tmpData1 = new triangle[tmpNum];
                for (int i = 0; i < tmpNum; ++i) {
                    tmpData1[i] = tmpData[i];
                }
                delete[] tmpData;
                data.push_back(std::pair<triangle*, int>(tmpData1, tmpNum));
            }
        }
        return false;
    }

};

Visibility getVisibility(Camera* cam, const AxisAlignedBox& bound) {
    if (bound.isNull())
        return NONE;
    Vector3 center = bound.getCenter();
    Vector3 halfSize = bound.getHalfSize();

    bool all_inside = true;

    for (uchar plane = 0; plane < 6; ++plane) {
        if (plane == FRUSTUM_PLANE_FAR && cam->getFarClipDistance() == 0)
            continue;

        Plane::Side side = cam->getFrustumPlane(plane).getSide(bound);

        if (side == Plane::NEGATIVE_SIDE)
            return NONE;
        if (side == Plane::BOTH_SIDE)
            all_inside = false;
    }

    if (all_inside)
        return FULL;
    else
        return PARTIAL;
}


class BasicTutorial1
    : public ApplicationContext
    , public InputListener
{
public:
    BasicTutorial1();
    virtual ~BasicTutorial1() {}

    void setup();
    bool frameStarted(const FrameEvent& evt) {
        ApplicationContext::frameStarted(evt);
        //帧时间
        count = (count + 1) % 60;
        if (count == 0) {
            time = getRoot()->getTimer()->getMilliseconds();
        }

        m_Mouse->capture();
        m_Keyboard->capture();
        OIS::MouseState mousePos = m_Mouse->getMouseState();
        float rotateSpeed = 90 / 300.0;

        if (mousePos.buttonDown(OIS::MB_Left)){
            float rx = mousePos.X.rel, ry = mousePos.Y.rel;
            //不跟随front和up的代码
            Quaternion camRotate = Quaternion(Degree(rotateSpeed * Math::Sqrt(rx * rx + ry * ry)), Vector3(-ry, 0, -rx));
            //
            camNode->setOrientation(viewCenter);
            camNode->setPosition(camRotate * camNode->getPosition());
            camNode->rotate(camRotate, Node::TS_WORLD);
            viewCenter = camNode->getOrientation();
            front = camRotate * front;
            up = camRotate * up;
            //camNode->rotate(camRotate, Node::TS_WORLD);
        }
        else {
            x += mousePos.X.rel; y += mousePos.Y.rel;
            camNode->setOrientation(viewCenter); camNode->setDirection(front, Node::TS_WORLD);
            camNode->yaw(-Degree(rotateSpeed * x));
            camNode->pitch(-Degree(rotateSpeed * y));
        }
        
        Quaternion xRotate = Quaternion(-Degree(rotateSpeed * x), up);
        Vector3 right = front.crossProduct(up);
        Quaternion yRotate = Quaternion(-Degree(rotateSpeed * y), xRotate * right);
        Vector3 tfront = yRotate*xRotate*front, tup = yRotate*xRotate * up;
        
        if (m_Keyboard->isKeyDown(OIS::KC_ESCAPE)) {
            getRoot()->queueEndRendering();
        }

        float moveSpeed = 50; Vector3 tright = tfront.crossProduct(tup);
        if (m_Keyboard->isKeyDown(OIS::KC_A)) {
            camNode->setPosition(camNode->getPosition() - moveSpeed * evt.timeSinceLastFrame * tright);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_D)) {
            camNode->setPosition(camNode->getPosition() + moveSpeed * evt.timeSinceLastFrame * tright);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_Q)) {
            camNode->setPosition(camNode->getPosition() + moveSpeed * evt.timeSinceLastFrame * tup);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_E)) {
            camNode->setPosition(camNode->getPosition() - moveSpeed * evt.timeSinceLastFrame * tup);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_W)) {
            camNode->setPosition(camNode->getPosition() + moveSpeed * evt.timeSinceLastFrame * tfront);
        }
        if (m_Keyboard->isKeyDown(OIS::KC_S)) {
            camNode->setPosition(camNode->getPosition() - moveSpeed * evt.timeSinceLastFrame * tfront);
        }

        if (m_Keyboard->isKeyDown(OIS::KC_C)) {
            std::pair<triangle*, int> p = kdTree->getData(cam);
            object->clear();
            object->begin("projectFace", RenderOperation::OT_TRIANGLE_LIST);
            for (int i = 0; i < p.second; ++i) {
                for (int j = 0; j < 3; j++) {
                    object->position(p.first[i].node[j].pos);
                    object->textureCoord(p.first[i].node[j].uv);
                }
            }
            //处理p中的triangle*造成的内存泄漏
            object->end();
            ogreNode->detachAllObjects();
            ogreNode->attachObject(object);
            panel->setParamValue("faceNum", std::to_string(p.second));

        }

        return true;
    };
    bool frameEnded(const FrameEvent& evt) {
        ApplicationContext::frameEnded(evt);
        if (count == 59) {
            float renderTime = (getRoot()->getTimer()->getMilliseconds() - time) / 60.0;
            panel->setParamValue("render time per frame", "\n" + std::to_string(renderTime) + "ms");
        }
        return true;
    }
private:
    bool flag = true;
    SceneNode* ogreNode;
    float time = 0;
    ParamsPanel* panel;
    int count = 0;
    OIS::Mouse* m_Mouse;
    OIS::Keyboard* m_Keyboard;
    SceneNode* camNode;
    Camera* cam;
    int x = 0, y = 0;
    Vector3 front, up;
    ManualObject* object;
    KDTree* kdTree;
    Quaternion viewCenter;
};


BasicTutorial1::BasicTutorial1()
    : ApplicationContext("OgreTutorialApp")
{
}


void BasicTutorial1::setup()
{
    // do not forget to call the base first
    ApplicationContext::setup();
    addInputListener(this);

    // get a pointer to the already created root
    Root* root = getRoot();
    SceneManager* scnMgr = root->createSceneManager();
    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    scnMgr->addRenderQueueListener(getOverlaySystem());
    TrayManager* mTrayMgr = new TrayManager("test", getRenderWindow());
    addInputListener(mTrayMgr);

    //! [camera]
    camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    cam->setFOVy(Degree(45));
    camNode->attachObject(cam); Vector3 camPos(0, -200, 0);
    camNode->setPosition(camPos);
    camNode->setDirection(Vector3(0, 1, 0), Node::TS_WORLD);
    camNode->setInitialState();
    viewCenter = camNode->getOrientation();
    front = Vector3(0, 1, 0); up = Vector3(0, 0, 1);

    // and tell it to render into the main window
    Viewport* vp = getRenderWindow()->addViewport(cam);
    //! [camera]
    vp->setBackgroundColour(ColourValue::Black);

    //ui
    mTrayMgr->showFrameStats(TL_TOPLEFT); mTrayMgr->toggleAdvancedFrameStats();
    std::vector<std::string> a;
    a.push_back("faceNum"); a.push_back("render time per frame"); a.push_back("");
    panel = mTrayMgr->createParamsPanel(TL_TOPLEFT, "time", 200, a);

    //mesh
    std::string objectFile[2] = { std::string("face_obj/Pasha_guard_head.obj"), std::string("Centurion_helmet_obj/Centurion_helmet.obj") };
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(objectFile[0], aiProcess_Triangulate | aiProcess_FlipUVs);
    count = 0;
    for (int i = 0; i < scene->mNumMeshes; i++) {
        count += scene->mMeshes[i]->mNumFaces;
    }
    triangle* faces = new triangle[count];
    //std::vector<triangle> faces;
    int tmpcount = 0;
    for (int i = 0; i < scene->mNumMeshes; i++) {
        for (int j = 0; j < scene->mMeshes[i]->mNumFaces; j++) {
            triangle face;
            for (int k = 0; k < 3; k++) {
                int index = scene->mMeshes[i]->mFaces[j].mIndices[k];
                Vector3 pos;
                pos.x = scene->mMeshes[i]->mVertices[index].x;
                pos.y = scene->mMeshes[i]->mVertices[index].y;
                pos.z = scene->mMeshes[i]->mVertices[index].z;
                Vector3 uv;
                uv.x = scene->mMeshes[i]->mTextureCoords[0][index].x;
                uv.y = scene->mMeshes[i]->mTextureCoords[0][index].y;
                uv.z = scene->mMeshes[i]->mTextureCoords[0][index].z;
                point node; node.pos = pos; node.uv = uv;
                face.node.push_back(node);
            }
            face.position = (face.node[0].pos + face.node[1].pos + face.node[2].pos) / 3.0;
            faces[tmpcount] = face;
            tmpcount++;
        }
    }
    kdTree = new KDTree(faces, count);
    object = scnMgr->createManualObject();
    ogreNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    std::pair<triangle*, int> p = kdTree->getData(cam);
    //material
    aiString str;
    scene->mMaterials[scene->mMeshes[0]->mMaterialIndex]->GetTexture(aiTextureType_DIFFUSE, 0, &str);
    //std::string textureFile = std::string("face_obj/")+std::string(str.C_Str());
    MaterialPtr material = MaterialManager::getSingleton().create("projectFace", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material->setReceiveShadows(false);
    Pass* pass = material->getTechnique(0)->getPass(0);
    pass->setLightingEnabled(false);
    pass->createTextureUnitState()->setTextureName(str.C_Str());
    object->clear();
    object->begin(material, RenderOperation::OT_TRIANGLE_LIST);
    for (int i = 0; i < p.second; ++i) {
        for (int j = 0; j < 3; j++) {
            object->position(p.first[i].node[j].pos);
            object->textureCoord(p.first[i].node[j].uv);    
        }
    }
    object->end();
    ogreNode->detachAllObjects();
    ogreNode->attachObject(object);
    panel->setParamValue("faceNum", std::to_string(p.second));
    

    //ui
    OIS::ParamList pl;
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;
    getRenderWindow()->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
    //pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND")));
    //pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
    //pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
    //pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
    OIS::InputManager* m_InputManager = OIS::InputManager::createInputSystem(pl);
    m_Mouse = static_cast<OIS::Mouse*>(m_InputManager->createInputObject(OIS::OISMouse, false));
    m_Keyboard = static_cast<OIS::Keyboard*>(m_InputManager->createInputObject(OIS::OISKeyboard, false));
    unsigned int width, height, depth;
    int top, left;
    getRenderWindow()->getMetrics(width, height, depth, left, top);
    const OIS::MouseState& ms = m_Mouse->getMouseState();
    ms.width = width;
    ms.height = height;
}


int main(int argc, char** argv)
{
    try
    {
        BasicTutorial1 app;
        app.initApp();
        app.getRoot()->startRendering();
        app.closeApp();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error occurred during execution: " << e.what() << '\n';
        return 1;
    }

    return 0;
}

//! [fullsource]
