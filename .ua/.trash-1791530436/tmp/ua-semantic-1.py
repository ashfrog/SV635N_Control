import ast, json, math, pathlib

ROOT = pathlib.Path('M:/GitHub/SV635N/SV635N_Control')
UA = ROOT / '.ua'
ex = json.loads((UA/'tmp/ua-file-extract-results-1.json').read_text(encoding='utf-8-sig'))
batch = next(b for b in json.loads((UA/'intermediate/batches.json').read_text(encoding='utf-8-sig'))['batches'] if b['batchIndex']==1)
ranges = json.loads((UA/'tmp/ua-method-ranges-1.json').read_text())

file_info = {
'app.py': ('Legacy Tk desktop frontend for direct-NIC pysoem control, device selection, dry runs, single moves, and persistent manual target updates. A single worker owns EtherCAT while queued events update the interface and cooperative stop controls cleanup.', ['legacy','entry-point','tkinter','manual-control','event-handler']),
'cli.py': ('Optional legacy command-line frontend that scans a dedicated EtherCAT adapter and selects motors by unique drive aliases. It verifies communication without enabling by default; --run requests a bounded relative move.', ['legacy','entry-point','cli','dry-run','ethercat']),
'continuous_control.py': ('Legacy persistent local PP controller extending core._Session with a latest-target queue, immediate updates during motion, and encoder rollover tracking. It holds enabled axes while idle and closes command submission before restoring hardware parameters.', ['legacy','manual-control','motion-control','state-machine','ethercat']),
'core.py': ('Legacy direct-NIC pysoem controller for discovery and validated CiA402 profile-position motion. Sessions verify drive identity, PDO/DC/watchdog settings, exchange cyclic feedback with bounded recovery, and disable drives before restoring temporary parameters.', ['legacy','ethercat','motion-control','cia402','hardware-backend']),
'platform_control.py': ('Legacy UDP bridge for three motor-angle targets and a persistent pysoem session, using hello/state and session/seq/enable/targets_deg messages. This older UE protocol is incompatible with the current versioned UDPServer API.', ['legacy','udp','ue-integration','motion-control','state-machine']),
'test_app_continuous.py': ('Tk flow regressions using the real local continuous session with an emulated EtherCAT master. They cover target editing, relative updates, live sliders, cancellation, single-move compatibility, and disabling after stop.', ['test','tkinter','manual-control','regression','hardware-emulation']),
'test_continuous_control.py': ('Hardware-free regressions for local latest-target queues and legacy EtherCAT sessions. They exercise numeric validation, bounded PDO recovery, immediate retargeting, idle enable, cleanup, and accumulated travel across encoder rollovers.', ['test','ethercat','manual-control','regression','hardware-emulation']),
'test_platform_control.py': ('Shared fake EtherCAT slave/master and hardware-free regressions for the legacy three-axis UDP bridge and CiA402 session. Tests cover peer ownership, sequence validation, latched disable, watchdog faults, position wrapping, and parameter restoration.', ['test','hardware-emulation','udp','cia402','legacy']),
}
class_info = {
'App': ('Tk desktop controller that manages motor selection, target editors, sliders, and asynchronous legacy EtherCAT operations. It serializes hardware work and consumes queued progress, status, and final reports.', ['component','tkinter','manual-control','event-handler']),
'EtherCATController': ('Public legacy pysoem facade for adapter discovery, protected device scanning, and bounded move execution. Each operation opens and closes the master under a per-adapter lock.', ['hardware-backend','ethercat','legacy','service']),
'_Session': ('Direct-NIC EtherCAT session implementing drive validation, temporary PP configuration, 1 kHz PDO exchange, and CiA402 move handshakes. Its lifecycle always attempts disable verification and parameter restoration before returning a report.', ['ethercat','cia402','state-machine','cleanup']),
'_ManualSession': ('Persistent manual controller derived from _Session that enables once and accepts the newest immutable target without waiting for previous arrival. It tracks unwrapped encoder motion and bounded histories, then closes the queue during cleanup.', ['manual-control','motion-control','state-machine','encoder-rollover']),
'CommandInbox': ('Thread-safe latest-target mailbox for the legacy three-axis protocol, validating session, sequence, finite bounded targets, and heartbeat freshness. Authenticated disable permanently latches the stop condition even when packets arrive out of order.', ['udp','validation','latest-target','watchdog']),
'UDPBridge': ('Background JSON/UDP transport for the legacy UE protocol that binds the first hello sender to the session and publishes state snapshots. Malformed traffic is ignored while accepted command errors and socket failures latch the mailbox stop.', ['udp','ue-integration','legacy','transport']),
'_ContinuousSession': ('Legacy three-axis _Session subclass that waits for an authenticated target before enabling, then applies latest absolute motor-angle targets. It retains enabled state between updates and enforces heartbeat, drive-state, and configured travel limits.', ['legacy','ue-integration','state-machine','watchdog']),
'AppTests': ('Tk integration tests using patched discovery and a fake master to exercise persistent enable, target editing, slider updates, cancellation, and single-move behavior.', ['test','tkinter','integration','hardware-emulation']),
'QueueTests': ('Validation and projection regressions for immutable latest-target MotionQueue commands, including finite positive profiles, atomic rejection, queue replacement, and closed sessions.', ['test','validation','latest-target','regression']),
'CommunicationTests': ('PDO recovery regressions using a fake master to ensure incomplete frames do not advance feedback or setpoint edges. They check bounded retries, cooperative stop, recovery timing, and alarm handling.', ['test','ethercat','fault-injection','regression']),
'FakeSlave': ('Emulates supported SV635N identity, SDO parameters, PDO mappings, watchdog/DC registers, and CiA402 feedback. Setpoint rising edges update wrapped encoder positions and recorded targets without physical hardware.', ['test','hardware-emulation','cia402','data-model']),
'FakeMaster': ('Minimal pysoem-compatible fake master holding four FakeSlave instances and expected WKC. It propagates bus states, exchanges simulated PDOs, and records close state for cleanup assertions.', ['test','hardware-emulation','ethercat','fixture']),
'InboxTests': ('Legacy UDP mailbox and bridge regressions for ordering, target validation, session rejection, heartbeat expiry, first-peer ownership, and permanent disable.', ['test','udp','validation','watchdog']),
}
method_summaries = {
'App.__init__':'Initializes the Tk window, persisted adapter settings, single hardware executor, event queue, target state, and polling callbacks.',
'App._style':'Defines Tk widget typography, colors, selected rows, and distinct run/stop button styles.',
'App._layout':'Builds adapter discovery, motor selection, motion profiles, target sliders, continuous-enable controls, and log widgets with event bindings.',
'App.update_summary':'Validates the displayed profile and describes signed single-move duration or immediate relative retargeting behavior.',
'App.update_buttons':'Derives widget availability from selection, busy state, acknowledged continuous readiness, and cooperative stop state.',
'App.refresh_adapters':'Enumerates pysoem adapters, selects the saved or first adapter, and updates discovery status and available actions.',
'App.adapter_changed':'Clears discovered devices, selection, targets, and readiness when the chosen adapter changes, then refreshes the controls.',
'App.submit':'Schedules one hardware operation on the single worker, replaces its stop token, and reports success or failure through the GUI event queue.',
'App.scan':'Persists the selected adapter, clears stale device state, and schedules an asynchronous scan that leaves motors disabled.',
'App.toggle':'Toggles a motor row selection while rejecting unsupported motor parameters and changes during active hardware operations.',
'App.run_motion':'Validates selected-axis motion parameters and submits a dry run or bounded single move; ready continuous sessions instead receive relative retargets.',
'App.toggle_continuous':'Creates a validated latest-target queue and starts persistent local control, or requests stop when continuous enable is turned off.',
'App.sync_slider':'Synchronizes the slider with selected target offsets, optionally recenters its display, and suppresses callbacks during programmatic changes.',
'App.apply_slider_range':'Validates finite slider center and span, then changes the display range while suppressing target submission.',
'App.slider_changed':'Applies a finite shared target to selected axes, submitting only changed targets in a ready continuous session and otherwise editing display values.',
'App.edit_target':'Places an entry editor over a motor target cell and binds commit/cancel events while preserving selection state.',
'App.queue_relative':'Validates a signed incremental angle, atomically updates all controlled targets relative to the planned position, and refreshes target displays.',
'App.send_targets':'Commits target edits and submits per-axis absolute offsets, or submits zeros to return to the captured enable origin.',
'App.stop':'Sets the worker stop event, closes the command queue, cancels target editing, and disables controls pending hardware cleanup.',
'App.poll':'Consumes queued phase, readiness, command, status, and completion events to update the Tk interface. It guards late readiness after stop and waits for safe completion before closing.',
'EtherCATController._discover':'Discovers 1–16 supported SV635N devices in PRE-OP, rejects already-enabled or incompatible drives, and reads identity, gearing, motor, and status data.',
'EtherCATController.scan':'Locks the adapter, opens a temporary master, discovers devices without enabling, and closes the master in finally.',
'EtherCATController.execute':'Validates nonempty unique move requests against the discovery snapshot and runs a _Session under the adapter lock.',
'_Session.__init__':'Allocates the pysoem master and lifecycle flags, indexes selected moves, and prepares report, trace, and optional journal paths.',
'_Session.setup':'Revalidates the device snapshot, supported motor/PDO/DC/watchdog configuration, and native profile limits before applying temporary drive settings. It journals originals and verifies OP communication while keeping all motors disabled.',
'_Session.tick':'Stages PDO outputs and performs a paced exchange with bounded WKC recovery using identical controlwords and targets. It validates feedback, drive state, travel, and host lateness, then emits throttled status.',
'_Session.motion':'Enables selected axes through CiA402 transitions and triggers one relative PP move with acknowledgement, arrival, and disable verification.',
'_Session.cleanup':'Attempts quick stop and full disable verification, returns the bus to PRE-OP, disables DC synchronization, and restores saved temporary parameters before the save policy.',
'_Session.run':'Runs setup and optional motion, converts cancellation/faults to reports, always invokes cleanup, and writes trace/journal files after hardware operation.',
'_ManualSession.__init__':'Creates placeholder profile moves for selected axes and initializes bounded trace/command history plus persistent-control report counters.',
'_ManualSession.motion':'Enables axes once without moving and implements acknowledgement/reset handshakes for newest absolute PP targets. It replaces active targets immediately, tracks unwrapped arrival, and computes per-command timeouts with braking margin.',
'CommandInbox.__init__':'Validates target bounds and heartbeat timeout and allocates a unique legacy protocol session, synchronization lock, latest slot, and latched stop event.',
'CommandInbox.accept':'Authenticates the session and validates sequence, boolean enable, and exactly three finite bounded targets; newer targets replace the slot while disable latches permanently.',
'UDPBridge.__enter__':'Binds the UDP socket and starts a daemon receive thread, closing the socket if transport startup fails.',
'UDPBridge.publish':'Updates synchronized feedback from phase/status/result events, including selected axes and verified disabled state after cleanup.',
'UDPBridge._receive_loop':'Handles bounded JSON packets from the first hello peer, accepts legacy commands, and sends periodic state responses while latching authenticated-command or transport failures.',
'_ContinuousSession.motion':'Waits for a fresh legacy target before enabling, captures origins, and exchanges immediate absolute three-axis PP setpoints through acknowledgement/reset states.',
'AppTests.setUp':'Patches adapter discovery and local control, creates a Tk window with fake devices, and marks three selected motors ready for UI tests.',
'AppTests.test_edit_targets_relative_origin_and_disable':'Exercises target cell editing, relative updates, return-to-origin, idle enable, and queue clearing plus disable after switching continuous control off.',
'AppTests.test_single_motion_keeps_its_original_behavior':'Verifies a GUI single move still triggers once per selected axis and finishes with motors disabled and continuous mode inactive.',
'AppTests.test_live_slider_controls_checked_motors_and_range_changes_do_not_move':'Verifies live slider updates reach only selected axes, display range changes submit no motion, large offsets work, and stop blocks later updates.',
'AppTests.test_slider_selection_change_and_mixed_targets_do_not_send':'Checks that changing selection or synchronizing mixed target displays does not submit motion, while user slider changes set a common controlled target.',
'AppTests.test_disabled_slider_only_edits_target':'Checks slider changes while disabled only edit targets, continuous enable begins at zero without commands, and later slider motion succeeds.',
'QueueTests.test_profile_has_no_software_upper_or_lower_range':'Accepts finite positive speed/acceleration profiles over very wide magnitudes and rejects zero, negative, boolean, nonfinite, and invalid values.',
'QueueTests.test_latest_relative_projection_and_atomic_rejection':'Checks readiness gating, immutable command targets, relative projection from planned targets, atomic invalid-command rejection, and single latest-slot replacement.',
'QueueTests.test_finiteness_latest_slot_and_closed_session':'Rejects malformed targets, verifies repeated relative updates occupy one slot, and ensures closed sessions reject further submission while fresh queues reset state.',
'CommunicationTests.session':'Builds a fake legacy _Session with expected WKC and explicitly seeded axis outputs/feedback for focused PDO exchange tests.',
'CommunicationTests.test_transient_timeout_and_partial_wkc_retry_without_new_target_edge':'Injects incomplete PDO frames and verifies identical setpoints are retried once, partial feedback is discarded, and motion is not triggered twice.',
'CommunicationTests.test_persistent_wkc_failure_is_bounded':'Checks persistent receive failure stops within five exchanges and neither updates feedback nor reports successful recovery.',
'CommunicationTests.test_stop_prevents_retry_and_slow_recovery_still_stops':'Verifies stop requested during receive prevents another exchange and a recovery attempt exceeding the time budget still fails closed.',
'CommunicationTests.test_alarm_after_recovery_is_not_ignored':'Injects a transient transport failure followed by valid feedback carrying an alarm and verifies the recovered frame still faults the session.',
'SessionTests.test_transient_wkc_during_target_ack_continues_and_restores':'Checks a transient WKC failure during continuous acknowledgement recovers without duplicate targets and leaves the bus clean after stop.',
'SessionTests.assert_clean':'Asserts all drives are disabled, the master is closed, selected-axis parameters are restored, and unselected drives never receive motion targets.',
'SessionTests.test_high_profile_and_tiny_positive_profile_use_native_parameters':'Verifies large and tiny finite positive motion profiles convert to native counts with a minimum value of one and restore correctly.',
'SessionTests.test_native_overflow_rejected_before_parameter_writes_or_enable':'Injects native 32-bit profile overflows and verifies validation rejects them before drive writes or enable while closing the master.',
'SessionTests.test_enable_holds_three_axes_and_idle_stays_enabled':'Verifies enabling holds captured positions, several absolute/relative targets execute with per-axis direction conversion, and an idle continuous session remains enabled until stop.',
'SessionTests.test_stop_during_first_move_clears_later_commands':'Checks stopping during a started command clears later queued targets, prevents their execution, and restores the drives.',
'SessionTests.test_updates_while_moving_without_waiting_for_arrival':'Emulates gradual motion and verifies immediate forward/reverse retargeting supersedes older commands without disabling or waiting for arrival.',
'SessionTests.test_fault_and_ack_timeout_disable_and_restore':'Injects communication, path, enable-state, and acknowledgement faults and checks the continuous session disables and restores selected drives.',
'SessionTests.test_pre_cancel_never_enables':'Verifies an already-set stop token prevents any motor target or enable-triggered movement and completes cancellation cleanup.',
'SessionTests.test_unlimited_travel_and_feedback_across_multiple_encoder_rollovers':'Runs hundreds of relative commands across multiple signed encoder rollovers and verifies large positive/negative unwrapped feedback and final return to origin.',
'FakeSlave.__init__':'Seeds supported identity, initial wrapped positions, native motion parameters, exact PDO maps, and saved originals for deterministic hardware-free tests.',
'FakeSlave.exchange':'Decodes output PDOs and emulates CiA402 state, acknowledgement, and arrival bits, applying a wrapped target only on a new-setpoint rising edge.',
'InboxTests.test_validation_and_stale_session':'Rejects stale sessions and malformed, nonfinite, boolean, oversized, or out-of-bound three-axis targets without publishing a latest command.',
'InboxTests.test_udp_handshake_peer_and_fault':'Uses loopback sockets to verify hello establishes the peer, stranger commands are ignored, valid commands update the inbox, and invalid owned commands latch stop.',
'SessionTests.test_watchdog_wkc_and_limit_fail_closed':'Injects heartbeat expiry, WKC failure, configured travel violation, and local stop into the legacy three-axis session and verifies disable, restoration, and master close.',
'SessionTests.test_persistent_motion_wrap_direction_stop_and_restore':'Verifies persistent three-axis targets honor per-axis direction and encoder wrapping, unchanged heartbeat targets do not restart motion, and stop restores parameters without intermediate disable cycles.',
}
function_info = {
'data_folder': ('Chooses a writable settings/log directory next to the script or frozen executable, falling back to LOCALAPPDATA after a write probe fails.', ['utility','filesystem','configuration']),
'read': ('Reads an integer from a drive SDO with up to three transport attempts, rejecting empty data and immediately propagating SDO protocol errors.', ['ethercat','sdo','retry']),
'mapping': ('Expands a drive PDO assignment and its referenced mapping objects into the exact list of mapped entries.', ['ethercat','pdo','validation']),
'displacement': ('Computes a signed modular 32-bit encoder displacement across rollover boundaries.', ['utility','encoder-rollover','motion-control']),
'main': ('Parses legacy direct-NIC motion options, scans and matches unique drive aliases, installs cooperative Ctrl-C stop, and returns the single-session report with dry-run default.', ['entry-point','cli','legacy']),
'run_continuous': ('Validates queued axis orders and runs the persistent manual session under the adapter lock, closing the queue regardless of outcome.', ['entry-point','manual-control','cleanup']),
'run_three_axis': ('Validates exactly three ordered distinct motors, configures the legacy target mailbox and UDP bridge, and runs a locked persistent session with combined feedback callbacks.', ['entry-point','legacy','ue-integration','udp']),
}
nodes=[];edges=[];edge_keys=set();node_by_id={};method_owners={}
weights={'contains':1.0,'exports':.8,'imports':.7,'calls':.8,'inherits':.9,'tested_by':.5}
def edge(source,target,kind):
    key=(source,target,kind)
    if source!=target and key not in edge_keys:
        edge_keys.add(key);edges.append(dict(source=source,target=target,type=kind,direction='forward',weight=weights[kind]))
def add(path,name,typ,summary,tags,complexity,start=None,end=None):
    ident=typ+':'+path+(':'+name if typ!='file' else '')
    node=dict(id=ident,type=typ,name=name,filePath=path,summary=summary,tags=tags,complexity=complexity)
    if start is not None:node['lineRange']=[start,end]
    assert ident not in node_by_id,ident
    nodes.append(node);node_by_id[ident]=node
    if typ!='file':edge('file:'+path,ident,'contains')
    return ident
for f in ex['results']:
    path=f['path'];summary,tags=file_info[path]
    add(path,path.rsplit('/',1)[-1],'file',summary,tags,'complex' if f['nonEmptyLines']>200 else 'moderate' if f['nonEmptyLines']>=50 else 'simple')
    exported={e['name'] for e in f.get('exports',[])}
    for fn in f.get('functions',[]):
        if fn['endLine']-fn['startLine']+1>=10 or fn['name'] in exported:
            desc,ftags=function_info[fn['name']]
            ident=add(path,fn['name'],'function',desc,ftags,'moderate' if fn['endLine']-fn['startLine']>20 else 'simple',fn['startLine'],fn['endLine'])
            if fn['name'] in exported:edge('file:'+path,ident,'exports')
    for cls in f.get('classes',[]):
        name=cls['name']
        if name=='SessionTests':
            if path=='test_platform_control.py':desc='Legacy three-axis and single-move regression suite verifying latched stop, watchdog faults, direction/wrapping, cleanup, and restoration with an emulated master.'
            else:desc='Persistent manual session regressions for native profile conversion, idle enable, immediate retargeting, fault cleanup, cancellation, and unwrapped multi-revolution feedback.'
            ctags=['test','motion-control','fault-injection','regression']
        else:desc,ctags=class_info[name]
        ident=add(path,name,'class',desc,ctags,'complex' if cls['endLine']-cls['startLine']>200 else 'moderate',cls['startLine'],cls['endLine'])
        if name in exported:edge('file:'+path,ident,'exports')
        for method in cls['methods']:
            canonical=name+'.'+method;m=ranges[path][canonical]
            if m['endLine']-m['startLine']+1<10:continue
            assert canonical in method_summaries,canonical
            mtags=['test','regression','hardware-emulation'] if path.startswith('test_') else ['event-handler','tkinter','manual-control'] if path=='app.py' else ['ethercat','motion-control','state-machine'] if name in ('_Session','_ManualSession','EtherCATController','_ContinuousSession') else ['udp','validation','legacy']
            mid=add(path,canonical,'function',method_summaries[canonical],mtags,'complex' if m['endLine']-m['startLine']>60 else 'moderate' if m['endLine']-m['startLine']>=20 else 'simple',m['startLine'],m['endLine'])
            method_owners[mid]=name
            edge(ident,mid,'contains')
    for target in batch['batchImportData'][path]:edge('file:'+path,'file:'+target,'imports')

edge('class:continuous_control.py:_ManualSession','class:core.py:_Session','inherits')
edge('class:platform_control.py:_ContinuousSession','class:core.py:_Session','inherits')
for prod,test in [('app.py','test_app_continuous.py'),('continuous_control.py','test_app_continuous.py'),('continuous_control.py','test_continuous_control.py'),('core.py','test_app_continuous.py'),('core.py','test_continuous_control.py'),('core.py','test_platform_control.py'),('platform_control.py','test_platform_control.py')]:edge('file:'+prod,'file:'+test,'tested_by')
edge('file:control_common.py','file:test_continuous_control.py','tested_by')

# Use deterministic extracted calls, and only resolve explicit project symbols or
# verified local/inherited method owners. External libraries never become edges.
symbol_files={}
for n in nodes:
    if n['type'] in ('function','class') and '.' not in n['name']:
        symbol_files.setdefault(n['name'],[]).append(n['id'])
external={'MotionQueue':'class:control_common.py:MotionQueue','Move':'class:control_common.py:Move','Device':'class:control_common.py:Device','adapter_lock':'function:control_common.py:adapter_lock'}
instances={
'app.py':{'controller':'EtherCATController'},
'cli.py':{'controller':'EtherCATController'},
'core.py':{'self.controller':'EtherCATController'},
'continuous_control.py':{'controller':'EtherCATController'},
'platform_control.py':{'bridge':'UDPBridge','inbox':'CommandInbox','self.inbox':'CommandInbox','session':'_ContinuousSession'},
'test_app_continuous.py':{'self.window':'App','self.controller':'EtherCATController'},
'test_continuous_control.py':{'controller':'EtherCATController','session':'_Session','slave':'FakeSlave'},
'test_platform_control.py':{'controller':'EtherCATController','inbox':'CommandInbox','bridge':'UDPBridge','session':'_ContinuousSession','slave':'FakeSlave'},
}
owner_paths={'App':'app.py','EtherCATController':'core.py','_Session':'core.py','_ManualSession':'continuous_control.py','_ContinuousSession':'platform_control.py','CommandInbox':'platform_control.py','UDPBridge':'platform_control.py','FakeMaster':'test_platform_control.py','FakeSlave':'test_platform_control.py'}
for f in ex['results']:
    path=f['path'];local=[n for n in nodes if n['filePath']==path and n['type']=='function']
    for call in f.get('callGraph',[]):
        callers=[n for n in local if n['name'].split('.')[-1]==call['caller'] and n['lineRange'][0]<=call['lineNumber']<=n['lineRange'][1]]
        if len(callers)!=1:continue
        caller=callers[0];callee=call['callee'];target=None
        if callee in external:target=external[callee]
        elif callee in symbol_files:
            candidates=symbol_files[callee]
            own=[i for i in candidates if node_by_id[i]['filePath']==path]
            imported=[i for i in candidates if node_by_id[i]['filePath'] in batch['batchImportData'][path]]
            resolved=own or imported
            if len(resolved)==1:target=resolved[0]
        elif callee.startswith('self.') and callee.count('.')==1 and caller['id'] in method_owners:
            owner=method_owners[caller['id']];meth=callee.split('.')[-1]
            ident='function:'+path+':'+owner+'.'+meth
            if ident in node_by_id:target=ident
            elif owner in ('_ManualSession','_ContinuousSession'):
                inherited='function:core.py:_Session.'+meth
                if inherited in node_by_id:target=inherited
        elif '.' in callee:
            instance,meth=callee.rsplit('.',1)
            owner=instances.get(path,{}).get(instance)
            if instance in owner_paths:owner=instance
            if instance=='super()' and method_owners.get(caller['id']) in ('_ManualSession','_ContinuousSession'):owner='_Session'
            if owner in owner_paths:
                ident='function:'+owner_paths[owner]+':'+owner+'.'+meth
                if ident in node_by_id:target=ident
                elif owner in ('_ManualSession','_ContinuousSession'):
                    ident='function:core.py:_Session.'+meth
                    if ident in node_by_id:target=ident
        if target:edge(caller['id'],target,'calls')

# The source wraps these constructors in an immediate inherited run() call;
# extract-structure records the qualified expression instead of a bare method.
edge('function:core.py:EtherCATController.execute','function:core.py:_Session.run','calls')
edge('function:continuous_control.py:run_continuous','function:core.py:_Session.run','calls')

expected_imports=sum(len(v) for v in batch['batchImportData'].values())
assert sum(e['type']=='imports' for e in edges)==expected_imports
assert len(nodes)==len(node_by_id)
allowed_external=set(external.values())|{'file:control_common.py'}
for e in edges:
    assert e['source'] in node_by_id or e['source'] in allowed_external,e
    assert e['target'] in node_by_id or e['target'] in allowed_external,e

parts=math.ceil(max(len(nodes)/60,len(edges)/120))
paths=sorted(file_info);chunk=math.ceil(len(paths)/parts)
written=[]
for index,start in enumerate(range(0,len(paths),chunk),1):
    own_paths=set(paths[start:start+chunk]);pn=[n for n in nodes if n['filePath'] in own_paths];ids={n['id'] for n in pn}
    pe=[e for e in edges if e['source'] in ids]
    # A tested_by edge from a neighboring production file belongs with its test.
    pe += [e for e in edges if e['source'] not in node_by_id and e['target'] in ids]
    out=UA/'intermediate'/f'batch-1-part-{index}.json'
    out.write_text(json.dumps({'nodes':pn,'edges':pe},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    check=json.loads(out.read_text(encoding='utf-8'));assert check['nodes']==pn and check['edges']==pe
    written.append(str(out))
assert sum(len(json.loads(pathlib.Path(p).read_text(encoding='utf-8'))['nodes']) for p in written)==len(nodes)
assert sum(len(json.loads(pathlib.Path(p).read_text(encoding='utf-8'))['edges']) for p in written)==len(edges)
print(json.dumps({'outputs':written,'nodes':len(nodes),'edges':len(edges),'imports':expected_imports,'skipped':ex['filesSkipped'],'warnings':['Extractor method names lack positions; AST source ranges supplement significance filtering and method line ranges.']},indent=2))
