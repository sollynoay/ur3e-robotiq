
"use strict";

let IsProgramRunning = require('./IsProgramRunning.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetRobotMode = require('./GetRobotMode.js')
let Popup = require('./Popup.js')
let AddToLog = require('./AddToLog.js')
let RawRequest = require('./RawRequest.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let GetProgramState = require('./GetProgramState.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let IsInRemoteControl = require('./IsInRemoteControl.js')
let Load = require('./Load.js')

module.exports = {
  IsProgramRunning: IsProgramRunning,
  GetSafetyMode: GetSafetyMode,
  GetRobotMode: GetRobotMode,
  Popup: Popup,
  AddToLog: AddToLog,
  RawRequest: RawRequest,
  GetLoadedProgram: GetLoadedProgram,
  GetProgramState: GetProgramState,
  IsProgramSaved: IsProgramSaved,
  IsInRemoteControl: IsInRemoteControl,
  Load: Load,
};
