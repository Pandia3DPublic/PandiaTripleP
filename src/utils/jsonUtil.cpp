#include "jsonUtil.h"

using namespace std;

Json::Value readJsonFromDisk(const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs)
    {
        std::cout << "Couldn't open file for reading: " << path << endl;
        return Json::Value();
    }
    Json::Value json;
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, ifs, &json, &errs))
    {
        std::cout << "Json parsing from file failed: " << path << endl;
        std::cout << errs << std::endl;
        return Json::Value();
    }
    ifs.close();
    return json;
}

bool writeJsonToDisk(const std::string& path, const Json::Value& json)
{
    std::ofstream ofs(path);
    if (!ofs)
    {
        std::cout << "Couldn't open file for writing: " << path << endl;
        return false;
    }
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "    ";
    builder.settings_["precision"] = 10;

    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(json, &ofs);
    ofs.close();
    return true;
}

Json::Value StringToJson(const std::string &json_str)
{
    Json::Value json;
    std::string err;
    Json::CharReaderBuilder builder;
    const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    if (!reader->parse(json_str.c_str(), json_str.c_str() + json_str.length(),
                       &json, &err))
    {
        cout << "Failed to parse string to json: " << err << endl;
    }
    return json;
}

std::string JsonToString(const Json::Value json)
{
    return Json::writeString(Json::StreamWriterBuilder(), json);
}

bool setValueFromJson(const Json::Value& json, const std::string &key, double &value)
{
    if (json.isMember(key))
    {
        value = json[key].asDouble();
        return true;
    }
    else
    {
        cout << "Json key " << key << " doesn't exist!" << endl;
        return false;
    }
}

bool setValueFromJson(const Json::Value& json, const std::string &key, float &value)
{
    if (json.isMember(key))
    {
        value = json[key].asFloat();
        return true;
    }
    else
    {
        cout << "Json key " << key << " doesn't exist!" << endl;
        return false;
    }
}

bool setValueFromJson(const Json::Value& json, const std::string &key, int &value)
{
    if (json.isMember(key))
    {
        value = json[key].asInt();
        return true;
    }
    else
    {
        cout << "Json key " << key << " doesn't exist!" << endl;
        return false;
    }
}

bool setValueFromJson(const Json::Value& json, const std::string &key, bool &value)
{
    if (json.isMember(key))
    {
        value = json[key].asBool();
        return true;
    }
    else
    {
        cout << "Json key " << key << " doesn't exist!" << endl;
        return false;
    }
}

bool setValueFromJson(const Json::Value& json, const std::string &key, std::string &value)
{
    if (json.isMember(key))
    {
        value = json[key].asString();
        return true;
    }
    else
    {
        cout << "Json key " << key << " doesn't exist!" << endl;
        return false;
    }
}