#pragma once

Json::Value readJsonFromDisk(const std::string& path);
bool writeJsonToDisk(const std::string& path, const Json::Value& json);

Json::Value StringToJson(const std::string &json_str);
std::string JsonToString(const Json::Value json);
bool setValueFromJson(const Json::Value& json, const std::string &key, double &value);
bool setValueFromJson(const Json::Value& json, const std::string &key, float &value);
bool setValueFromJson(const Json::Value& json, const std::string &key, int &value);
bool setValueFromJson(const Json::Value& json, const std::string &key, bool &value);
bool setValueFromJson(const Json::Value& json, const std::string &key, std::string &value);