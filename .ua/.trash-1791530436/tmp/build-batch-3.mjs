import fs from 'node:fs';
const root='M:/GitHub/SV635N/SV635N_Control';
const batch=JSON.parse(fs.readFileSync(root+'/.ua/intermediate/batches.json','utf8').replace(/^\uFEFF/,'' )).batches.find(b=>b.batchIndex===3);
const extraction=JSON.parse(fs.readFileSync(root+'/.ua/tmp/ua-file-extract-results-3.json','utf8'));
const nodes=[],edges=[];
const weights={contains:1,exports:.8,imports:.7,calls:.8,inherits:.9,depends_on:.6,tested_by:.5,related:.5};
const edge=(source,target,type)=>{if(source!==target&&!edges.some(e=>e.source===source&&e.target===target&&e.type===type))edges.push({source,target,type,direction:'forward',weight:weights[type]});};
const node=(type,path,name,summary,tags,range)=>{const id=type==='file'?`file:${path}`:`${type}:${path}:${name}`;nodes.push({id,type,name,filePath:path,summary,tags,complexity:range?(range[1]-range[0]>100?'complex':range[1]-range[0]>35?'moderate':'simple'):'complex',...(range?{lineRange:range}:{})});if(type!=='file')edge('file:'+path,id,'contains');return id;};
const fileMeta={
'aps_backend.py':['Owns the ADLINK PCIe-8332 APS DLL lifecycle and validates SV635N topology, units, direction and endpoint sensors. Runs card-generated point-to-point motion with preflight checks, continuous feedback, directional limit escape, and verified stop/disable/parameter restoration.',['hardware-backend','ctypes','ethercat','motion-control','safety']],
'backend.py':['Starts the Windows tray or headless UDP backend, validates configuration and records rotating logs. Creates MotorService and UDPServer, launches the control UI in a separate process, and closes the server and hardware owner on exit.',['entry-point','configuration','system-tray','lifecycle']],
'control_common.py':['Defines hardware-independent device, motion profile, command and error types. Coordinates cross-process hardware exclusivity and a locked latest-target queue whose relative intent accumulates while pending commands are replaced.',['data-model','validation','concurrency','motion-queue']],
'motor_service.py':['Coordinates asynchronous scan and enable lifecycles, immutable status snapshots and run-scoped sequenced targets/heartbeats. A single-worker executor owns hardware access, while a watchdog requests stops and schedules idle sensor reads; failures latch until an explicit scan.',['service','hardware-owner','single-worker','watchdog','lifecycle']],
'test_aps_backend.py':['Exercises APS ABI bindings, configuration, topology and motion using a fake API with injected faults and sensor changes. Tests directional endpoint protection, stop/disable/restoration and real UDP integration, including proof that APS access stays on one hardware thread.',['test','fake-hardware','safety','udp-integration','regression']],
'test_udp_ui.py':['Tests the Tk UDP frontend against injected legacy hardware and a local server. Verifies selection, hidden-window heartbeats, isolated tray client launch and endpoint displays that distinguish trigger, conflict, unavailable and stale feedback.',['test','gui','udp-integration','sensor-feedback']],
'udp_server.py':['Implements versioned UDP JSON RPC with exclusive session leases, authentication, request deduplication and monotonic control sequencing. Routes requests to MotorService, publishes snapshots, and requests a stop when the socket worker fails or the server closes.',['udp-server','rpc','authentication','deduplication','telemetry']]
};
const descriptions={
AsyncCall:'ctypes layout for an APS asynchronous call handle, return status and mode; used in the point-to-point ABI signature.',
ModuleInfo:'ctypes mirror of the APS field-bus module structure, including device identity, axis mapping and IO module arrays.',
APSLibrary:'Loads the matching-bitness APS DLL from SDK locations, assigns ctypes ABI signatures and turns negative return codes or invalid numeric feedback into ControlError.',
APSDevice:'Immutable extension of Device with APS axis/slave identity, position units and configured extension/retraction sensor directions.',
PCIe8332Controller:'Owns one configured PCIe-8332 card on one hardware thread, validates SV635N devices, executes continuous card trajectories and restores owned hardware state during cleanup.',
ControlError:'Shared operational error type for invalid requests, hardware faults and failed safety checks.',
Stopped:'ControlError subtype used to unwind intentional continuous-motion shutdown separately from hardware faults.',
Device:'Immutable scanned device record carrying chain position, identity, drive state, gear ratio and confirmed motor direction.',
Move:'Immutable motion profile that validates motor selection, finite speed/acceleration and the confirmed encoder, and estimates triangular or trapezoidal rest-to-rest duration.',
MotionCommand:'Immutable numbered vector of absolute target angles with its estimated duration.',
MotionQueue:'Thread-safe latest-target queue with sorted unique motor orders, readiness/closed gates, duration limits and accumulated relative targets.',
MotorService:'Publishes locked lifecycle state and serializes scan, motion and cleanup on a single hardware executor, with heartbeat expiry and idle sensor monitoring.',
FakeAPS:'Stateful fake APS API that records calls and hardware thread identities and models axes, bus state, sensor PDOs, mappings and injected motion/cleanup failures.',
BindingTests:'Validates ctypes sizes/signatures, DLL load/export diagnostics and backend/APS configuration rejection rules.',
ControllerTests:'Tests fake-card scanning, axis/unit/direction conversion, endpoint interlocks, fault handling, persistent enable and complete stop/disable/restoration.',
APSUDPTests:'Exercises real UDP requests and heartbeats against a fake APS card, checking endpoint escape, conflict faults, idle sensor telemetry and one-thread hardware ownership.',
UITests:'Exercises the UDP Tk control UI and tray launch isolation with a fake EtherCAT controller, including hidden-window continuity and stale sensor rendering.',
UDPServer:'Owns the UDP socket, exclusive control lease, bounded request cache and telemetry stream, and delegates validated requests to the hardware-owning service.'
};
const topSummaries={
'aps_backend.py:validate_options':'Normalizes APS defaults and rejects unknown keys, invalid card IDs/timing/units and malformed or overlapping extension/retraction sensor mappings.',
'backend.py:load_config':'Loads optional JSON backend settings, normalizes APS options and derives the PCIe card adapter while rejecting unknown or inconsistent configuration.',
'backend.py:run_tray':'Runs a system tray icon whose default action launches a separate UDP frontend process and whose stop/exit actions request service shutdown.',
'backend.py:main':'Parses launch flags, sets up rotating logs and signal handling, starts the service and UDP server, and closes both after tray/headless execution or failure.',
'control_common.py:adapter_lock':'Acquires a process-local lock and, on Windows, a named mutex keyed by adapter to exclude concurrent sessions; releases all handles in a finally block.',
'test_aps_backend.py:output':'Writes a ctypes output parameter through the original pointee for fake APS API responses.',
'udp_server.py:encode':'Serializes compact UTF-8 JSON and rejects nonfinite numeric output.',
'udp_server.py:decode':'Rejects oversized or non-object JSON datagrams and NaN/Infinity before request handling.'
};
// Method ranges supplement the bundled parser's string-only classes[].methods output.
// Names originate solely from that extraction; ranges were checked against complete source.
const methodMeta={
'aps_backend.py':{
'APSLibrary.__init__':[67,91,'Loads a bitness-matched APS DLL and binds every required function signature, reporting missing SDK libraries and exports.'],
'APSLibrary.call':[93,99,'Invokes a bound APS function and raises a diagnostic error on a negative return, with an EtherCAT configuration hint for -4012.'],
'APSLibrary.value':[101,106,'Reads a typed APS output parameter and rejects nonfinite double feedback.'],
'PCIe8332Controller.__init__':[177,189,'Validates card configuration and prepares a lazy APS factory, lifecycle flags, saved parameters and hardware-thread ownership.'],
'PCIe8332Controller._open':[205,252,'Acquires APS exclusivity, initializes the expected card, safely adopts or starts the bus, saves board settings and restores partial initialization on failure.'],
'PCIe8332Controller._configure_unwired_emg':[254,273,'For explicitly unwired emergency input only, verifies all servos off, saves and flips common EMG polarity and confirms the input releases.'],
'PCIe8332Controller._devices':[283,342,'Enumerates mapped single-axis SV635N slaves, checks online/servo-off state and SDO identity/gearing, and validates unit overrides and opposite endpoint DI assignments.'],
'PCIe8332Controller.scan':[344,346,'Opens the configured APS lifecycle and returns freshly validated device topology without enabling motors.'],
'PCIe8332Controller.read_inputs':[348,392,'Reads 32-bit 60FD cyclic PDO feedback and native IO limits into per-device endpoint states, preserving unavailable feedback as unknown.'],
'PCIe8332Controller.close':[401,420,'Stops the owned field bus, restores saved board parameters, closes APS and releases the exclusivity context while accumulating cleanup errors.'],
'PCIe8332Controller.run_continuous':[422,695,'Revalidates topology, enables only selected axes, converts targets to card trajectories and continuously checks faults and directional endpoint limits. Finally stops every owned axis, verifies disable, restores axis parameters and writes a bounded run report.']},
'control_common.py':{
'Move.validate_profile':[76,89,'Rejects invalid physical motor orders, nonfinite/nonpositive speed and acceleration, and unconfirmed encoder types.'],
'MotionQueue.__init__':[134,146,'Validates sorted unique motor orders and motion profiles and initializes the locked pending-target state.'],
'MotionQueue.submit':[160,185,'Checks readiness, finite angle vectors and duration, accumulates relative intent, replaces pending commands with the newest numbered target and records the planned vector.']},
'motor_service.py':{
'MotorService.__init__':[18,39,'Validates heartbeat timeout and initializes the hardware owner, state locks, single-worker executor and watchdog thread.'],
'MotorService.snapshot':[44,61,'Returns a deep-copied status document including device identities, sensor configuration, planned targets, axis enable feedback and cleanup results.'],
'MotorService.scan':[75,88,'Rejects busy/shutdown scans, validates the adapter, clears stale state and queues a hardware scan.'],
'MotorService._scan':[90,106,'Scans and reads sensors on the hardware executor; publishes devices on success or closes hardware and latches fault on failure.'],
'MotorService.enable':[108,132,'Validates selected scanned motors and motion profiles, starts a new run ID and heartbeat epoch, then queues continuous motion on the sole hardware worker.'],
'MotorService._event':[134,148,'Processes hardware readiness, axis/sensor telemetry, blocked-limit messages and phase updates under the service lock.'],
'MotorService._run':[150,174,'Runs native or injected legacy continuous control, publishes a bounded result and transitions to idle only after an intentional verified clean stop; otherwise latches fault.'],
'MotorService.command':[176,223,'Validates run and per-channel sequence, refuses expired heartbeats or unsafe endpoint targets, submits changed targets and refreshes heartbeat only for accepted requests.'],
'MotorService.stop':[225,233,'Requests shutdown of an active run, closes its target queue and enters stopping until the hardware worker completes parameter restoration.'],
'MotorService._watchdog':[235,245,'Requests a stop when an active heartbeat expires and schedules idle sensor reads on the existing hardware executor.'],
'MotorService._poll_inputs':[247,265,'Refreshes idle input telemetry on the hardware thread and publishes unavailable states if reads fail, always clearing the pending-poll flag.'],
'MotorService.close':[267,279,'Stops monitoring and motion, queues hardware lifecycle closure and waits for the executor so stop/disable/restoration cannot be abandoned.']},
'udp_server.py':{
'UDPServer.__init__':[36,49,'Requires authentication for nonlocal binding and initializes exclusive owner/session state, control sequencing and request caches.'],
'UDPServer.start':[51,65,'Binds a UDP socket with exclusive-address use where available, starts a receive thread and closes the socket on startup failure.'],
'UDPServer.handle':[81,165,'Validates protocol/authentication/session and deduplication, enforces control ordering and dispatches lease, scan, enable, target, heartbeat, disable and release requests.'],
'UDPServer._receive':[175,199,'Decodes and handles datagrams, tolerates timeout/ICMP and malformed input, periodically publishes snapshots and stops motion on receive-worker failure.'],
'UDPServer.close':[201,207,'Requests motor stop, closes the socket and joins the receive worker during server shutdown.']},
'test_udp_ui.py':{
'UITests.test_ui_uses_udp_selection_and_hides_without_stopping':[40,78,'Checks UDP axis selection and target control, safe range changes, continued heartbeats while hidden, usable layout and explicit disable.'],
'UITests.test_backend_tray_launches_separate_client_without_tk':[80,100,'Mocks tray launch to verify a separate UdpControl process receives the actual UDP port and the backend initializes no injected hardware.'],
'UITests.test_sensor_display_distinguishes_trigger_unknown_and_no_retraction_sensor':[102,127,'Checks trigger styling, raw DI display, absent retraction sensor and unavailable/stale feedback rendering.'],
'UITests.test_dual_endpoint_display_and_stale_feedback':[129,159,'Checks opposite endpoint labels, conflict faults and separate unavailable/stale rendering for configured retraction sensors.']},
'test_aps_backend.py':{
'FakeAPS.__init__':[26,54,'Creates reproducible card, bus, axis, sensor and parameter state with knobs for injected faults and delayed disable feedback.'],
'FakeAPS.call':[56,153,'Simulates APS lifecycle, topology, SDO/PDO, IO, motion and stop functions while recording calls and thread identity.'],
'BindingTests.test_abi_and_negative_error_diagnostic':[162,178,'Verifies ctypes ABI sizes and signatures and the -4012 configuration diagnostic using a mocked DLL.'],
'BindingTests.test_backend_config':[188,208,'Checks default/derived card adapter and rejects legacy adapter, invalid APS types, unit mappings and overlapping endpoint inputs.'],
'ControllerTests.run_motion':[219,230,'Builds a motion queue and stop event, dispatches scripted actions after continuous readiness and collects the controller run report.'],
'ControllerTests.test_mapping_gear_direction_and_moving_override':[232,256,'Checks gear-scaled target and velocity conversion, reversed motor direction, moving target replacement, selected-axis ownership and cleanup restoration.'],
'ControllerTests.test_sensor_bits_and_unknown_feedback':[264,283,'Checks native limit/DI bit decoding, unconfigured states and unavailable feedback for incorrect PDO bit length or failed reads.'],
'ControllerTests.test_bad_sensor_function_or_axis_prevents_scan':[285,293,'Rejects unsupported driver DI function assignments and endpoint mappings to nonexistent APS axes.'],
'ControllerTests.test_single_limit_blocks_outward_and_allows_inward_both_directions':[295,316,'Checks outward rejection and permitted reverse escape for either configured native endpoint direction.'],
'ControllerTests.test_dual_sensor_states_and_driver_configuration':[318,342,'Checks dual endpoint clear/trigger/conflict states, missing feedback and validation of opposite driver DI functions and axis existence.'],
'ControllerTests.test_dual_limits_block_either_end_and_allow_escape_with_ui_direction_conversion':[344,371,'Exercises both endpoint directions, native/UI direction conversion and connected/unwired mapping settings while allowing only escape targets.'],
'ControllerTests.test_both_endpoints_triggered_prevent_enable_and_fault_during_run':[373,386,'Checks that conflicting endpoint sensors prevent servo enable and produce a fully disabled runtime fault.'],
'ControllerTests.test_retraction_trigger_stops_motion_and_clearing_does_not_resume_old_target':[388,414,'Triggers a retraction limit during motion and checks immediate stop with persistent enable and no automatic old-target restart after clearing.'],
'ControllerTests.test_trigger_during_motion_stops_and_does_not_resume_on_clear':[416,446,'Checks extension-trigger stopping, no automatic restart after clearing and later acceptance of an explicit reverse target.'],
'ControllerTests.test_configured_sensor_failure_prevents_enable_and_stops_run':[448,460,'Verifies unavailable configured endpoint feedback prevents preflight enable and faults/fully disables an already active run.'],
'ControllerTests.test_native_limit_stop_allows_escape_but_other_astp_is_fault':[462,488,'Allows reverse escape after a recognized directional native limit stop but treats an unrelated abnormal stop code as a fault.'],
'ControllerTests.test_arrival_keeps_servo_enabled_until_explicit_stop':[490,511,'Checks completed target accounting and feedback while the servo remains enabled until an explicit stop.'],
'ControllerTests.test_unwired_limits_disable_only_selected_mapping_and_restore':[522,531,'Checks that only selected unwired limit mappings are changed and that cleanup restores their original values.'],
'ControllerTests.test_unwired_emg_releases_native_input_and_restores_on_close':[549,559,'Checks explicit unwired EMG polarity release and restoration on controller close.'],
'ControllerTests.test_unwired_emg_still_active_does_not_enable':[561,569,'Checks a failed polarity release prevents any servo enable and restores the saved board setting.'],
'ControllerTests.test_released_emg_historical_stop_clears_on_explicit_target':[571,584,'Checks historical stopped ASTP state after unwired EMG release permits only a new explicit motion target.'],
'ControllerTests.test_runtime_faults_and_stop_fallback':[600,620,'Injects alarm/offline/bus/abnormal/target failures and failed deceleration stops, checking emergency-stop fallback, disable results and lifecycle invalidation.'],
'ControllerTests.test_card_discovery_and_close_on_failed_start':[629,641,'Checks wrong/missing card and failed bus start trigger APS closure and restoration of original board settings.'],
'ControllerTests.test_boot_auto_connected_bus_and_explicit_eni_regeneration':[643,654,'Verifies a valid already-operational bus is adopted and explicit ENI regeneration orders stop, scan and start correctly.'],
'APSUDPTests.test_udp_dual_limits_escape_each_end_and_latch_conflict':[665,707,'Exercises both endpoint escape directions over UDP and verifies simultaneous endpoint activation latches a fully disabled fault on one hardware thread.'],
'APSUDPTests.test_udp_rejects_extension_at_limit_and_accepts_retraction':[709,749,'Checks unsafe endpoint UDP targets preserve target sequence and issue no move, while a reverse target is accepted with independent heartbeats.'],
'APSUDPTests.test_idle_inputs_update_over_udp_without_enabling_any_motor':[751,783,'Checks idle sensor transitions and unavailable feedback propagate through UDP while no servo or trajectory call occurs.'],
'APSUDPTests.test_real_udp_target_and_heartbeat_stop_with_single_hardware_thread':[785,816,'Checks scan/enable/target UDP flow, selected APS axes, heartbeat expiry stopping, single hardware-thread access and restored bus/board lifecycle.']}
};
const symbolsByFile={};
for(const result of extraction.results){
 const path=result.path;const [summary,tags]=fileMeta[path];node('file',path,path,summary,tags);nodes.at(-1).complexity=result.nonEmptyLines>200?'complex':result.nonEmptyLines>50?'moderate':'simple';
 for(const target of batch.batchImportData[path])edge('file:'+path,'file:'+target,'imports');
 const exported=new Set((result.exports||[]).map(e=>e.name));
 const emitted=[];
 for(const fn of result.functions||[]){if(fn.endLine-fn.startLine+1>=10||exported.has(fn.name)){const id=node('function',path,fn.name,topSummaries[path+':'+fn.name],tags.slice(0,4),[fn.startLine,fn.endLine]);if(exported.has(fn.name))edge('file:'+path,id,'exports');emitted.push({id,name:fn.name,start:fn.startLine,end:fn.endLine});}}
 for(const cls of result.classes||[]){if(cls.methods.length>=2||cls.endLine-cls.startLine+1>=20||exported.has(cls.name)){const id=node('class',path,cls.name,descriptions[cls.name],tags.slice(0,4),[cls.startLine,cls.endLine]);if(exported.has(cls.name))edge('file:'+path,id,'exports');}
  for(const method of cls.methods){const key=cls.name+'.'+method;const meta=methodMeta[path]?.[key];if(!meta)continue;const [start,end,summary]=meta;const id=node('function',path,key,summary,tags.slice(0,4),[start,end]);edge('class:'+path+':'+cls.name,id,'contains');emitted.push({id,name:method,owner:cls.name,start,end});}
 }
 symbolsByFile[path]=emitted;
}
const idSet=new Set(nodes.map(n=>n.id));
const call=(path,from,targetPath,to)=>{const source='function:'+path+':'+from;const target=(to.startsWith('class:')?to:'function:'+targetPath+':'+to);if(!idSet.has(source))throw Error('Unknown call source '+source);if(!idSet.has(target))throw Error('Unknown call target '+target);edge(source,target,'calls');};
// Resolve self calls within their owning class and constructors/top-level calls using extracted call positions.
const receiverMaps={
'backend.py':{service:['motor_service.py','MotorService'],server:['udp_server.py','UDPServer']},
'motor_service.py':{'self.hardware':['aps_backend.py','PCIe8332Controller'],controller:['aps_backend.py','PCIe8332Controller'],'self.commands':['control_common.py','MotionQueue'],commands:['control_common.py','MotionQueue']},
'udp_server.py':{'self.service':['motor_service.py','MotorService']},
'test_aps_backend.py':{'self.c':['aps_backend.py','PCIe8332Controller'],service:['motor_service.py','MotorService'],server:['udp_server.py','UDPServer'],api:['aps_backend.py','APSLibrary'],commands:['control_common.py','MotionQueue'],q:['control_common.py','MotionQueue']},
'test_udp_ui.py':{'self.service':['motor_service.py','MotorService'],'self.server':['udp_server.py','UDPServer']}
};
for(const result of extraction.results){const path=result.path;for(const c of result.callGraph||[]){
 const caller=symbolsByFile[path].filter(s=>c.lineNumber>=s.start&&c.lineNumber<=s.end).sort((a,b)=>(a.end-a.start)-(b.end-b.start))[0];if(!caller)continue;
 let target;
 if(c.callee.startsWith('self.')&&caller.owner){const candidate=`function:${path}:${caller.owner}.${c.callee.slice(5)}`;if(idSet.has(candidate))target=candidate;}
 if(!target&& !c.callee.includes('.')){for(const neighbor of [path,...batch.batchImportData[path]]){for(const type of ['function','class']){const candidate=`${type}:${neighbor}:${c.callee}`;if(idSet.has(candidate))target=candidate;}}}
 if(!target){for(const [receiver,[targetPath,owner]] of Object.entries(receiverMaps[path]||{})){if(c.callee.startsWith(receiver+'.')){const candidate=`function:${targetPath}:${owner}.${c.callee.slice(receiver.length+1)}`;if(idSet.has(candidate))target=candidate;}}}
 if(target)edge(caller.id,target,'calls');
}}
// Explicit executor scheduling and callbacks are behavioral links, not synchronous call-graph inference.
for(const [from,to] of [['MotorService.scan','MotorService._scan'],['MotorService.enable','MotorService._run'],['MotorService._watchdog','MotorService._poll_inputs']])edge('function:motor_service.py:'+from,'function:motor_service.py:'+to,'depends_on');
call('motor_service.py','MotorService._run','aps_backend.py','PCIe8332Controller.run_continuous');
edge('function:aps_backend.py:PCIe8332Controller.run_continuous','function:motor_service.py:MotorService._event','related');
call('motor_service.py','MotorService._scan','aps_backend.py','PCIe8332Controller.scan');
edge('class:aps_backend.py:APSDevice','class:control_common.py:Device','inherits');
edge('class:control_common.py:Stopped','class:control_common.py:ControlError','inherits');
for(const path of ['test_aps_backend.py','test_udp_ui.py'])for(const target of batch.batchImportData[path])if(!target.startsWith('test_'))edge('file:'+target,'file:'+path,'tested_by');
// Cross-batch references use only symbols provided by neighborMap.
edge('class:test_aps_backend.py:APSUDPTests','class:UdpControl/client.py:MotorClient','depends_on');
edge('class:test_udp_ui.py:UITests','class:UdpControl/debug_ui.py:DebugWindow','depends_on');
edge('class:test_udp_ui.py:UITests','class:core.py:EtherCATController','depends_on');
edge('class:test_udp_ui.py:UITests','class:test_platform_control.py:FakeMaster','depends_on');
const importCount=edges.filter(e=>e.type==='imports').length;
if(importCount!==Object.values(batch.batchImportData).reduce((n,a)=>n+a.length,0))throw Error('Import loss');
const external=new Set();for(const arr of Object.values(batch.batchImportData))for(const p of arr)external.add('file:'+p);for(const arr of Object.values(batch.neighborMap))for(const n of arr){external.add('file:'+n.path);for(const s of n.symbols)for(const type of ['function','class'])external.add(type+':'+n.path+':'+s);}
for(const e of edges){if(!idSet.has(e.source)&&!external.has(e.source))throw Error('Unknown source '+e.source);if(!idSet.has(e.target)&&!external.has(e.target))throw Error('Unknown target '+e.target);}
const parts=Math.ceil(Math.max(nodes.length/60,edges.length/120));const paths=batch.files.map(f=>f.path).sort();const groupSize=Math.ceil(paths.length/parts);const written=[];
for(let i=0;i<parts;i++){const files=new Set(paths.slice(i*groupSize,(i+1)*groupSize));const partNodes=nodes.filter(n=>files.has(n.filePath));const sources=new Set(partNodes.map(n=>n.id));const graph={nodes:partNodes,edges:edges.filter(e=>sources.has(e.source))};const path=root+'/.ua/intermediate/'+(parts===1?'batch-3.json':`batch-3-part-${i+1}.json`);fs.writeFileSync(path,JSON.stringify(graph,null,2));JSON.parse(fs.readFileSync(path,'utf8'));written.push({path,nodes:graph.nodes.length,edges:graph.edges.length});}
console.log(JSON.stringify({written,totalNodes:nodes.length,totalEdges:edges.length,importCount,filesSkipped:extraction.filesSkipped,warning:'Bundled extraction omits method ranges and owner qualifications. Significant method ranges were verified in source; executor scheduling uses depends_on and callback transfer uses related. Cross-part references validate against the complete batch node union as permitted by partition Step C.'},null,2));
