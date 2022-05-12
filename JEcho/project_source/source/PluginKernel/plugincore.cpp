// -----------------------------------------------------------------------------
//    ASPiK Plugin Kernel File:  plugincore.cpp
//
/**
    \file   plugincore.cpp
    \author Will Pirkle
    \date   17-September-2018
    \brief  Implementation file for PluginCore object
    		- http://www.aspikplugins.com
    		- http://www.willpirkle.com
*/
// -----------------------------------------------------------------------------
#include "plugincore.h"
#include "plugindescription.h"

/**
\brief PluginCore constructor is launching pad for object initialization

Operations:
- initialize the plugin description (strings, codes, numbers, see initPluginDescriptors())
- setup the plugin's audio I/O channel support
- create the PluginParameter objects that represent the plugin parameters (see FX book if needed)
- create the presets
*/
PluginCore::PluginCore()
{
    // --- describe the plugin; call the helper to init the static parts you setup in plugindescription.h
    initPluginDescriptors();

    // --- default I/O combinations
	// --- for FX plugins
	if (getPluginType() == kFXPlugin)
	{
		addSupportedIOCombination({ kCFMono, kCFMono });
		addSupportedIOCombination({ kCFMono, kCFStereo });
		addSupportedIOCombination({ kCFStereo, kCFStereo });
	}
	else // --- synth plugins have no input, only output
	{
		addSupportedIOCombination({ kCFNone, kCFMono });
		addSupportedIOCombination({ kCFNone, kCFStereo });
	}

	// --- for sidechaining, we support mono and stereo inputs; auxOutputs reserved for future use
	addSupportedAuxIOCombination({ kCFMono, kCFNone });
	addSupportedAuxIOCombination({ kCFStereo, kCFNone });

	// --- create the parameters
    initPluginParameters();

    // --- create the presets
    initPluginPresets();
}

/**
\brief create all of your plugin parameters here

\return true if parameters were created, false if they already existed
*/
bool PluginCore::initPluginParameters()
{
	if (pluginParameterMap.size() > 0)
		return false;

    // --- Add your plugin parameter instantiation code bewtween these hex codes
	// **--0xDEA7--**

	PluginParameter* piParam = nullptr;

	piParam = new PluginParameter(0, "delaytime", "ms", controlVariableType::kDouble, 1.000000, 2000.000000, 500.000000, taper::kLinearTaper);
	piParam->setBoundVariable(&delaytime, boundVariableType::kDouble);
	addPluginParameter(piParam);


	piParam = new PluginParameter(1, "Sync", "OFF, ON", "OFF");
	piParam->setBoundVariable(&sync, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(2, "Delay Setting", "Full, FullD, FullT, Half, HalfD, HalfT, Quarter, QuarterD, QuarterT, Eighth, EighthD, EighthT, Sixteenth, SixteenthD, SixteenthT, Thirtytwo", "Full");
	piParam->setBoundVariable(&delaySetting, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(3, "feedback", "%", controlVariableType::kDouble, 1.000000, 100.000000, 50.000000, taper::kLinearTaper);
	piParam->setBoundVariable(&feedback, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(4, "wet", "%", controlVariableType::kDouble, 0.000000, 1.000000, 0.500000, taper::kLinearTaper);
	piParam->setBoundVariable(&wet, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(5, "gain", "dB", controlVariableType::kDouble, -60.000000, 12.000000, 0.000000, taper::kLinearTaper);
	piParam->setBoundVariable(&gain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(6, "saturation", "dB", controlVariableType::kDouble, 0.000000, 48.000000, 0.000000, taper::kLinearTaper);
	piParam->setBoundVariable(&saturation, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(7, "satbypass", "OFF, ON", "OFF");
	piParam->setBoundVariable(&satbypass, boundVariableType::kInt);
	addPluginParameter(piParam);

	piParam = new PluginParameter(8, "delayoffset", "ms", controlVariableType::kDouble, 0.000000, 50.000000, 0.000000, taper::kLinearTaper);
	piParam->setBoundVariable(&delayoffset, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(9, "fc", "Hz", controlVariableType::kDouble, 20.000000, 20000.000000, 1000.000000, taper::kVoltOctaveTaper);
	piParam->setBoundVariable(&fc, boundVariableType::kDouble);
	addPluginParameter(piParam);

	piParam = new PluginParameter(10, "drivevolume", "dB", controlVariableType::kDouble, -60.000000, 0.000000, -24.000000, taper::kLinearTaper);
	piParam->setBoundVariable(&drivevolume, boundVariableType::kDouble);
	addPluginParameter(piParam);
	
    
    
	// **--0xEDA5--**
   
    // --- BONUS Parameter
    // --- SCALE_GUI_SIZE
    PluginParameter* piParamBonus = new PluginParameter(SCALE_GUI_SIZE, "Scale GUI", "tiny,small,medium,normal,large,giant", "normal");
    addPluginParameter(piParamBonus);

	// --- create the super fast access array
	initPluginParameterArray();

    return true;
}

/**
\brief initialize object for a new run of audio; called just before audio streams

Operation:
- store sample rate and bit depth on audioProcDescriptor - this information is globally available to all core functions
- reset your member objects here

\param resetInfo structure of information about current audio format

\return true if operation succeeds, false otherwise
*/
bool PluginCore::reset(ResetInfo& resetInfo)
{
    // --- save for audio processing
    audioProcDescriptor.sampleRate = resetInfo.sampleRate;
    audioProcDescriptor.bitDepth = resetInfo.bitDepth;

	auto sr = getSampleRate();

	

	

	BufferLength = (unsigned int)(2 * (sr)) + 1;

	BufferL.createCircularBuffer(BufferLength);
	BufferR.createCircularBuffer(BufferLength);


	BufferL.flushBuffer();
	BufferR.flushBuffer();
    // --- other reset inits
    return PluginBase::reset(resetInfo);
	
	
}

/**
\brief one-time initialize function called after object creation and before the first reset( ) call

Operation:
- saves structure for the plugin to use; you can also load WAV files or state information here
*/
bool PluginCore::initialize(PluginInfo& pluginInfo)
{
	// --- add one-time init stuff here

	


	return true;
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- syncInBoundVariables when preProcessAudioBuffers is called, it is *guaranteed* that all GUI control change information
  has been applied to plugin parameters; this binds parameter changes to your underlying variables
- NOTE: postUpdatePluginParameter( ) will be called for all bound variables that are acutally updated; if you need to process
  them individually, do so in that function
- use this function to bulk-transfer the bound variable data into your plugin's member object variables

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::preProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
    // --- sync internal variables to GUI parameters; you can also do this manually if you don't
    //     want to use the auto-variable-binding
    syncInBoundVariables();

    return true;
}

//Method to update parameter values before each frame
void PluginCore::updateParameters(double delaytimesync) {

	//Get sample rate
	double sr = getSampleRate();

	//delay time ms
	double delaytimems = 0.0000000;

	//get delay time in ms from either raw ms value / from sync value using branchless bypass
	delaytimems = (delaytime * branchlessbypass[sync]) + (delaytimesync * sync);

	//times delaytime in ms by samples per ms to get delay in samples
	delaytimesamples = delaytimems * (sr / 1000);

	//times offset in ms by samples per ms to get offset in samples
	delayoffsetsamples = delayoffset * (sr / 1000);

	//create sample delay in right channel by summating the dt and offset
	delaysamplesright = delaytimesamples + delayoffsetsamples;

	//convert gain to raw 
	gaincooked = pow(10, gain/20);

	//convert saturation to raw
	saturationcooked = pow(10, saturation / 20);

	drivevolumecooked = pow(10, drivevolume / 20); 
	



	/*preparams = tubepre.getParameters();

	preparams.saturation = saturationcooked;

	preparams.*/

	//get filter params from object
	filterparams = FilterL.getParameters();

	//fill in filter params
	filterparams.algorithm = filterAlgorithm::kButterLPF2;
	filterparams.fc = fc;
	filterparams.boostCut_dB = 0.0;
	filterparams.Q = 0.707; 
	
	//apply filter params to filter objects
	FilterL.setParameters(filterparams);
	FilterR.setParameters(filterparams);
}

//function to calculate delay times when sync enabled 
double PluginCore::delayCalc(double BPM) {

	double quarterNoteDelay = 60.0 / BPM;
	double fullNoteDelay = quarterNoteDelay * 4;
	double fullNoteDotted = fullNoteDelay * 1.5;
	double fullNoteTriplet = fullNoteDelay / 1.5;
	double halfNoteDelay = quarterNoteDelay * 2;
	double halfNoteDotted = halfNoteDelay * 1.5;
	double halfNoteTriplet = halfNoteDelay / 1.5;
	double quarterNoteDotted = quarterNoteDelay * 1.5;
	double quarterNoteTriplet = quarterNoteDelay / 1.5;
	double eighthNoteDelay = quarterNoteDelay / 2.0;
	double eighthNoteDotted = eighthNoteDelay * 1.5;
	double eighthNoteTripletDelay = quarterNoteDelay / 3.0;
	double sixteenthNoteDelay = quarterNoteDelay / 4.0;
	double sixteenthNoteDotted = sixteenthNoteDelay * 1.5;
	double sixteenthNoteTripletDelay = quarterNoteDelay / 5.0;
	double thirtytwothNoteDelay = quarterNoteDelay / 6.0;




	double delayNotes[16] = { fullNoteDelay, fullNoteDotted, fullNoteTriplet, halfNoteDelay, halfNoteDotted, halfNoteTriplet, quarterNoteDelay, quarterNoteDotted, quarterNoteTriplet, eighthNoteDelay, eighthNoteDotted, eighthNoteTripletDelay, sixteenthNoteDelay, sixteenthNoteDotted, sixteenthNoteTripletDelay, thirtytwothNoteDelay };

	return (delayNotes[delaySetting] * 1000);
}

/**
\brief frame-processing method

Operation:
- decode the plugin type - for synth plugins, fill in the rendering code; for FX plugins, delete the if(synth) portion and add your processing code
- note that MIDI events are fired for each sample interval so that MIDI is tightly sunk with audio
- doSampleAccurateParameterUpdates will perform per-sample interval smoothing

\param processFrameInfo structure of information about *frame* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processAudioFrame(ProcessFrameInfo& processFrameInfo)
{

	//acquire delaytimes from host info
	double BPM = processFrameInfo.hostInfo->dBPM;

	//pass in bpm and return delay time in ms for sync 
	double delaytimesync = delayCalc(BPM);


    // --- fire any MIDI events for this sample interval
    processFrameInfo.midiEventQueue->fireMidiEvents(processFrameInfo.currentFrame);

	// --- do per-frame updates; VST automation and parameter smoothing
	doSampleAccurateParameterUpdates();

	updateParameters(delaytimesync);

    // --- FX Plugin:
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFMono)
    {
		// --- pass through code: change this with your signal processing
        double xn = processFrameInfo.audioInputFrame[0];

		double xnsat = tanhWaveShaper(xn, saturationcooked);

		double yn = BufferL.readBuffer(delaytimesamples);

		double dn = (((xn * branchlessbypass[satbypass])+(xnsat * satbypass * drivevolumecooked)) + (feedback / 100) * yn);
	
		dn = FilterL.processAudioSample(dn);

		BufferL.writeBuffer(dn);

		double output = (xn * (1.0f - wet) + yn * wet) * gaincooked;

		processFrameInfo.audioOutputFrame[0] = output;


        return true; /// processed
    }

    // --- Mono-In/Stereo-Out
	else if (processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
		processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
	{
		// --- pass through code: change this with your signal processing
		double xnl = processFrameInfo.audioInputFrame[0];
		double xnr = processFrameInfo.audioInputFrame[0];

		double xnsatl = tanhWaveShaper(xnl, saturationcooked);
		double xnsatr = tanhWaveShaper(xnr, saturationcooked);

		double ynl = BufferL.readBuffer(delaytimesamples);
		double ynr = BufferR.readBuffer(delaysamplesright);

		double dnl = (((xnl * branchlessbypass[satbypass]) + (xnsatl * satbypass * drivevolumecooked)) + (feedback / 100) * ynl);
		double dnr = (((xnr * branchlessbypass[satbypass]) + (xnsatr * satbypass * drivevolumecooked)) + (feedback / 100) * ynr);

		dnl = FilterL.processAudioSample(dnl);
		dnr = FilterR.processAudioSample(dnr);

		BufferL.writeBuffer(dnl);
		BufferR.writeBuffer(dnr);

		ynl = (xnl * (1.0f - wet) + ynl * wet) * gaincooked;
		ynr = (xnr * (1.0f - wet) + ynr * wet) * gaincooked;

		double output[2] = {ynl, ynr};

		processFrameInfo.audioOutputFrame[0] = output[0];
		processFrameInfo.audioOutputFrame[1] = output[1];

	


        return true; /// processed
    }

    // --- Stereo-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFStereo &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
		double xnl = processFrameInfo.audioInputFrame[0];
		double xnr = processFrameInfo.audioInputFrame[1];

		double xnsatl = tanhWaveShaper(xnl, saturationcooked);
		double xnsatr = tanhWaveShaper(xnr, saturationcooked);

		double ynl = BufferL.readBuffer(delaytimesamples);
		double ynr = BufferR.readBuffer(delaysamplesright);

		double dnl = (((xnl * branchlessbypass[satbypass]) + (xnsatl * satbypass * drivevolumecooked)) + (feedback / 100) * ynl);
		double dnr = (((xnr * branchlessbypass[satbypass]) + (xnsatr * satbypass * drivevolumecooked)) + (feedback / 100) * ynr);

		dnl = FilterL.processAudioSample(dnl);
		dnr = FilterR.processAudioSample(dnr);

		BufferL.writeBuffer(dnl);
		BufferR.writeBuffer(dnr);

		ynl = (xnl * (1.0f - wet) + ynl * wet) * gaincooked;
		ynr = (xnr * (1.0f - wet) + ynr * wet) * gaincooked;

		double output[2] = { ynl, ynr };

		processFrameInfo.audioOutputFrame[0] = output[0];
		processFrameInfo.audioOutputFrame[1] = output[1];

		

        return true; /// processed
    }

    return false; /// NOT processed
}


/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- updateOutBoundVariables sends metering data to the GUI meters

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
	// --- update outbound variables; currently this is meter data only, but could be extended
	//     in the future
	updateOutBoundVariables();

    return true;
}

/**
\brief update the PluginParameter's value based on GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- use base class helper
    setPIParamValue(controlID, controlValue);

    // --- do any post-processing
    postUpdatePluginParameter(controlID, controlValue, paramInfo);

    return true; /// handled
}

/**
\brief update the PluginParameter's value based on *normlaized* GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param normalizedValue the new control value in normalized form
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameterNormalized(int32_t controlID, double normalizedValue, ParameterUpdateInfo& paramInfo)
{
	// --- use base class helper, returns actual value
	double controlValue = setPIParamValueNormalized(controlID, normalizedValue, paramInfo.applyTaper);

	// --- do any post-processing
	postUpdatePluginParameter(controlID, controlValue, paramInfo);

	return true; /// handled
}

/**
\brief perform any operations after the plugin parameter has been updated; this is one paradigm for
	   transferring control information into vital plugin variables or member objects. If you use this
	   method you can decode the control ID and then do any cooking that is needed. NOTE: do not
	   overwrite bound variables here - this is ONLY for any extra cooking that is required to convert
	   the GUI data to meaninful coefficients or other specific modifiers.

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postUpdatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- now do any post update cooking; be careful with VST Sample Accurate automation
    //     If enabled, then make sure the cooking functions are short and efficient otherwise disable it
    //     for the Parameter involved
    /*switch(controlID)
    {
        case 0:
        {
            return true;    /// handled
        }

        default:
            return false;   /// not handled
    }*/

    return false;
}

/**
\brief has nothing to do with actual variable or updated variable (binding)

CAUTION:
- DO NOT update underlying variables here - this is only for sending GUI updates or letting you
  know that a parameter was changed; it should not change the state of your plugin.

WARNING:
- THIS IS NOT THE PREFERRED WAY TO LINK OR COMBINE CONTROLS TOGETHER. THE PROPER METHOD IS
  TO USE A CUSTOM SUB-CONTROLLER THAT IS PART OF THE GUI OBJECT AND CODE.
  SEE http://www.willpirkle.com for more information

\param controlID the control ID value of the parameter being updated
\param actualValue the new control value

\return true if operation succeeds, false otherwise
*/
bool PluginCore::guiParameterChanged(int32_t controlID, double actualValue)
{
	/*
	switch (controlID)
	{
		case controlID::<your control here>
		{

			return true; // handled
		}

		default:
			break;
	}*/

	return false; /// not handled
}

/**
\brief For Custom View and Custom Sub-Controller Operations

NOTES:
- this is for advanced users only to implement custom view and custom sub-controllers
- see the SDK for examples of use

\param messageInfo a structure containing information about the incoming message

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMessage(MessageInfo& messageInfo)
{
	// --- decode message
	switch (messageInfo.message)
	{
		// --- add customization appearance here
	case PLUGINGUI_DIDOPEN:
	{
		return false;
	}

	// --- NULL pointers so that we don't accidentally use them
	case PLUGINGUI_WILLCLOSE:
	{
		return false;
	}

	// --- update view; this will only be called if the GUI is actually open
	case PLUGINGUI_TIMERPING:
	{
		return false;
	}

	// --- register the custom view, grab the ICustomView interface
	case PLUGINGUI_REGISTER_CUSTOMVIEW:
	{

		return false;
	}

	case PLUGINGUI_REGISTER_SUBCONTROLLER:
	case PLUGINGUI_QUERY_HASUSERCUSTOM:
	case PLUGINGUI_USER_CUSTOMOPEN:
	case PLUGINGUI_USER_CUSTOMCLOSE:
	case PLUGINGUI_EXTERNAL_SET_NORMVALUE:
	case PLUGINGUI_EXTERNAL_SET_ACTUALVALUE:
	{

		return false;
	}

	default:
		break;
	}

	return false; /// not handled
}


/**
\brief process a MIDI event

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param event a structure containing the MIDI event data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMIDIEvent(midiEvent& event)
{
	return true;
}

/**
\brief (for future use)

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param vectorJoysickData a structure containing joystick data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::setVectorJoystickParameters(const VectorJoystickData& vectorJoysickData)
{
	return true;
}

/**
\brief use this method to add new presets to the list

NOTES:
- see the SDK for examples of use
- for non RackAFX users that have large paramter counts, there is a secret GUI control you
  can enable to write C++ code into text files, one per preset. See the SDK or http://www.willpirkle.com for details

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginPresets()
{
	// **--0xFF7A--**

	// **--0xA7FF--**

    return true;
}

/**
\brief setup the plugin description strings, flags and codes; this is ordinarily done through the ASPiKreator or CMake

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginDescriptors()
{
    pluginDescriptor.pluginName = PluginCore::getPluginName();
    pluginDescriptor.shortPluginName = PluginCore::getShortPluginName();
    pluginDescriptor.vendorName = PluginCore::getVendorName();
    pluginDescriptor.pluginTypeCode = PluginCore::getPluginType();

	// --- describe the plugin attributes; set according to your needs
	pluginDescriptor.hasSidechain = kWantSidechain;
	pluginDescriptor.latencyInSamples = kLatencyInSamples;
	pluginDescriptor.tailTimeInMSec = kTailTimeMsec;
	pluginDescriptor.infiniteTailVST3 = kVSTInfiniteTail;

    // --- AAX
    apiSpecificInfo.aaxManufacturerID = kManufacturerID;
    apiSpecificInfo.aaxProductID = kAAXProductID;
    apiSpecificInfo.aaxBundleID = kAAXBundleID;  /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.aaxEffectID = "aaxDeveloper.";
    apiSpecificInfo.aaxEffectID.append(PluginCore::getPluginName());
    apiSpecificInfo.aaxPluginCategoryCode = kAAXCategory;

    // --- AU
    apiSpecificInfo.auBundleID = kAUBundleID;   /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.auBundleName = kAUBundleName;

    // --- VST3
    apiSpecificInfo.vst3FUID = PluginCore::getVSTFUID(); // OLE string format
    apiSpecificInfo.vst3BundleID = kVST3BundleID;/* MacOS only: this MUST match the bundle identifier in your info.plist file */
	apiSpecificInfo.enableVST3SampleAccurateAutomation = kVSTSAA;
	apiSpecificInfo.vst3SampleAccurateGranularity = kVST3SAAGranularity;

    // --- AU and AAX
    apiSpecificInfo.fourCharCode = PluginCore::getFourCharCode();

    return true;
}

// --- static functions required for VST3/AU only --------------------------------------------- //
const char* PluginCore::getPluginBundleName() { return kAUBundleName; }
const char* PluginCore::getPluginName(){ return kPluginName; }
const char* PluginCore::getShortPluginName(){ return kShortPluginName; }
const char* PluginCore::getVendorName(){ return kVendorName; }
const char* PluginCore::getVendorURL(){ return kVendorURL; }
const char* PluginCore::getVendorEmail(){ return kVendorEmail; }
const char* PluginCore::getAUCocoaViewFactoryName(){ return AU_COCOA_VIEWFACTORY_STRING; }
pluginType PluginCore::getPluginType(){ return kPluginType; }
const char* PluginCore::getVSTFUID(){ return kVSTFUID; }
int32_t PluginCore::getFourCharCode(){ return kFourCharCode; }

