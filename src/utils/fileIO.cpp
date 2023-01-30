#include "fileIO.h"
#include "miscFunctions.h"
using namespace std;

std::string getStringforNumberedFile(int a)
{
    std::string out = std::to_string(a);
    while (out.length() < 6)
    {
        out = "0" + out;
    }
    return out;
}

void setFromIntrinsicFile(const string &filepath, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    string stringline;
    ifstream cameraIntrinsic(filepath);
    int linecounter = 0;
    if (cameraIntrinsic.is_open())
    {
        while (getline(cameraIntrinsic, stringline))
        {
            IntrinsicFileHelperFunction(stringline, linecounter, intrinsic);
            linecounter++;
        }
    }
    cameraIntrinsic.close();
}

// assigns intrinsic-data from .txt-File
void IntrinsicFileHelperFunction(string value, int lineCounter, open3d::camera::PinholeCameraIntrinsic &intrinsic)
{
    int row = lineCounter - 2;
    if (lineCounter == 0)
        intrinsic.width_ = stoi(value);
    if (lineCounter == 1)
        intrinsic.height_ = stoi(value);
    if (lineCounter > 1)
    {
        istringstream iss(value);
        for (int column = 0; column < 3; column++)
        {
            double matrixValue;
            iss >> matrixValue;
            intrinsic.intrinsic_matrix_(row, column) = matrixValue;
        }
    }
}

// checks if file exists
bool fileExists(const string &filename)
{
    ifstream file;
    file.open(filename);
    if (file)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool saveMatrixToDisc(string path, string name, const Eigen::Matrix4d &matrix)
{
    cout << "Saving matrix to " << path << "\n";
    ofstream file(path + name);
    if (file.is_open())
    {
        file << matrix << endl;
    }
    else
    {
        cout << "Writing Matrix to file failed! \n";
        return false;
    }
    file.close();
    return true;
}

bool saveVectorToDisc(std::string path, std::string name, const std::vector<float> vector)
{
    ofstream file(path + name);
    if (file.is_open())
    {
        for (auto &i : vector)
        {
            file << i << endl;
        }
    }
    else
    {
        cout << "Writing vector to file failed! \n";
        return false;
    }
    file.close();
    return true;
}

bool saveStringToDisc(std::string path, std::string name, const string &content)
{

    ofstream file(path + name);
    if (file.is_open())
    {
        file << content << endl;
    }
    else
    {
        cout << "Writing string to file failed! \n";
        return false;
    }
    file.close();
    return true;
}

Eigen::Matrix4d readMatrixFromDisc(const std::string &PathAndName)
{
    ifstream matrix(PathAndName);
    string line;
    std::string delimiter = " ";
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    if (matrix.is_open())
    {
        int lineNumber = 0;
        while (getline(matrix, line))
        {
            // cout << " line " << line << endl;
            int row = lineNumber % 4;
            std::string token;
            size_t pos;
            int col = 0;
            while ((pos = line.find(delimiter)) != std::string::npos)
            {
                token = line.substr(0, pos);
                if (token.compare(""))
                { // this stupid shit return wrong if strings are equal
                    //	cout << token;// << delimiter;
                    out(row, col) = stod(token);
                    line.erase(0, pos + delimiter.length());
                    col++;
                }
                else
                {
                    // cout << "token is empty" << token << "after \n";
                    line.erase(0, 1);
                }
            }
            // cout << line <<  endl;

            out(row, col) = stod(line); // last entry
            lineNumber++;
        }
    }
    else
    {
        cout << "Warning: Reading a matrix from file went wrong\n";
        return Eigen::Matrix4d::Identity();
    }
    return out;
}

std::string readStringFromDisc(const std::string &path, const std::string &name)
{
    ifstream input(path + name);
    string out;
    string line;
    if (input.is_open())
    {
        while (getline(input, line))
        {
            out = out + line;
        }
    }
    else
    {
        cout << "Warning: Reading a string from file went wrong\n";
        return string();
    }
    return out;
}

std::string getExtensionFromFilename(const std::string &filename)
{
    size_t pos = filename.find_last_of(".");
    if (pos != string::npos)
    {
        return filename.substr(pos + 1);
    }
    else
    {
        cout << "Warning: getExtensionFromFilename failed." << endl;
        return "";
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> readMesh(const string &path)
{
    auto out = std::make_shared<open3d::geometry::TriangleMesh>();
    if (open3d::io::ReadTriangleMesh(path, *out))
    {
        return out;
    }
    else
    {
        // cout << "Warning: Reading mesh failed!" << endl;
        return std::make_shared<open3d::geometry::TriangleMesh>();
    }
}

std::shared_ptr<open3d::geometry::PointCloud> readPcd(const string &path)
{
    auto out = std::make_shared<open3d::geometry::PointCloud>();
    open3d::io::ReadPointCloudOption params;
    if (open3d::io::ReadPointCloudFromPCD(path, *out, params))
    {
        return out;
    }
    else
    {
        // cout << "Warning: Reading pointcloud failed!" << endl;
        return std::make_shared<open3d::geometry::PointCloud>();
    }
}

// fill vector of model paths
std::vector<std::string> getFileNames(const std::string& folder)
{
    if (folder.empty())
    {
        cout << "warning: no file directory supplied" << endl;
    }
    string path = get_current_dir_name() + folder;
    std::vector<std::string> out;

    // dirent.h file handling to get files in directory
    DIR *dir;
    struct dirent *file;
    if ((dir = opendir(path.c_str())) != NULL)
    {
        /* print all the files and directories within directory */
        while ((file = readdir(dir)) != NULL)
        {
            if (!file->d_name || file->d_name[0] == '.')
            {
                continue; // skip everything that starts with a dot
            }
            // printf("%s\n", file->d_name);
            out.push_back(file->d_name);
        }
        closedir(dir);
    }
    else
    {
        cout << "warning: could not open directory: " << path << endl;
    }
    std::sort(out.begin(), out.end(), SortbyAlphabet);
    return out;
}
