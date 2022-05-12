// --- CMAKE generated variables for your plugin

#include "pluginstructures.h"

#ifndef _plugindescription_h
#define _plugindescription_h

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#define AU_COCOA_VIEWFACTORY_STRING STR(AU_COCOA_VIEWFACTORY_NAME)
#define AU_COCOA_VIEW_STRING STR(AU_COCOA_VIEW_NAME)

// --- AU Plugin Cocoa View Names (flat namespace) 
#define AU_COCOA_VIEWFACTORY_NAME AUCocoaViewFactory_B7E7CA74C53D3EE1865EF3267E84E1BE
#define AU_COCOA_VIEW_NAME AUCocoaView_B7E7CA74C53D3EE1865EF3267E84E1BE

// --- BUNDLE IDs (MacOS Only) 
const char* kAAXBundleID = "developer.aax.jecho.bundleID";
const char* kAUBundleID = "developer.au.jecho.bundleID";
const char* kVST3BundleID = "developer.vst3.jecho.bundleID";

// --- Plugin Names 
const char* kPluginName = "JEcho";
const char* kShortPluginName = "JEcho";
const char* kAUBundleName = "JEcho_AU";

// --- Plugin Type 
const pluginType kPluginType = pluginType::kFXPlugin;

// --- VST3 UUID 
const char* kVSTFUID = "{b7e7ca74-c53d-3ee1-865e-f3267e84e1be}";

// --- 4-char codes 
const int32_t kFourCharCode = 'JECH';
const int32_t kAAXProductID = 'JECH';
const int32_t kManufacturerID = 'ASPK';

// --- Vendor information 
const char* kVendorName = "JEPlugins";
const char* kVendorURL = "www.myplugins.com";
const char* kVendorEmail = "support@myplugins.com";

// --- Plugin Options 
const bool kWantSidechain = false;
const uint32_t kLatencyInSamples = 0;
const double kTailTimeMsec = 0.000000;
const bool kVSTInfiniteTail = false;
const bool kVSTSAA = false;
const uint32_t kVST3SAAGranularity = 1;
const uint32_t kAAXCategory = 0;

#endif
