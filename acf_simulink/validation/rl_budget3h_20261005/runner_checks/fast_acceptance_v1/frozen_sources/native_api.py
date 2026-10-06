"""Callable Simulink-generated plant; contains no independent Python physics.

build_library(build_dir) adapts GRT global storage to per-context pointers,
adds an output/update-boundary hook, and compiles the emitted C unchanged
otherwise. NativePlant/create/reset/step use physical SI action units.
Actor observations, physical scoring, and comparator actions are separate.
"""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
import subprocess

import numpy as np

_PTR = ct.POINTER(ct.c_double)
_VOID = ct.c_void_p

METRIC_NAMES = (
    "elapsed_s", "start_s", "end_s", "native_samples", "min_normal_N", "max_normal_N",
    "peak_normal_error_N", "mean_squared_normal_error_N2", "min_travel_m", "max_travel_m",
    "mean_squared_travel_error_m2", "peak_current_A", "peak_voltage_V", "mean_squared_current_A2",
    "voltage_clip_samples", "current_limit_samples", "force_outside_samples", "contact_loss",
    "travel_stop", "nonfinite", "terminated", "record_samples", "physical_tick", "last_normal_N",
)


@dataclass
class StepResult:
    obs: np.ndarray
    truth_last: np.ndarray
    comparator: np.ndarray
    metrics: dict

    def __getattr__(self, name):
        if name == "physical_failure":
            return bool(self.metrics["contact_loss"] or self.metrics["travel_stop"])
        if name in self.metrics:
            return self.metrics[name]
        raise AttributeError(name)


def _hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_library(build_dir: str | Path) -> Path:
    """Compile one native graph with all exact prepared scenario parameters."""
    folder = Path(build_dir).resolve()
    code = folder / "code/acf_rl_native_grt_rtw"
    emitted = code / "acf_rl_native.c"
    source = emitted.read_text()
    prefix_start = source.index("/* Block signals (default storage) */")
    prefix_end = source.index("static void rate_scheduler(void);", prefix_start)
    bindings = "\n".join([
        "/* Private multi-instance GRT storage bindings. */",
        "#define acf_rl_native_P (acf_active->parameters)",
        "#define acf_rl_native_B (acf_active->signals)",
        "#define acf_rl_native_DW (acf_active->states)",
        "#define acf_rl_native_X (acf_active->continuous)",
        "#define acf_rl_native_XDis (acf_active->disabled)",
        "#define acf_rl_native_U (acf_active->inputs)",
        "#define acf_rl_native_Y (acf_active->outputs)",
        "#define acf_rl_native_M (&acf_active->model)", "",
    ])
    adapted = source[:prefix_start] + bindings + source[prefix_end:]
    update = re.search(r"\n  if \(rtmIsMajorTimeStep\(acf_rl_native_M\)\) \{\n(?:[^\n]*\n){0,12}\s*/\* Update for TransportDelay:", adapted)
    if update is None:
        raise RuntimeError("Missing exact emitted native output/update boundary")
    hook = "\n  if (rtmIsMajorTimeStep(acf_rl_native_M)) acf_native_output_hook();\n"
    adapted = adapted[:update.start()] + hook + adapted[update.start():]
    adapted_path = folder / "acf_rl_native_context.c"
    adapted_path.write_text(adapted)
    # Prove the adapter made only the two specified packaging substitutions.
    recovered = adapted.replace(hook, "", 1).replace(bindings, source[prefix_start:prefix_end], 1)
    if recovered != source:
        raise AssertionError("Generated native computation changed")

    scenarios = json.loads((folder / "scenarios.json").read_text())
    mapping = json.loads((folder / "parameter_mapping.json").read_text())
    seeds = [m["name"] for m in mapping if m["name"].endswith("_Seed")]
    aux = ["static const int acf_bank_sensor_seeds[] = {" + ",".join(str(int(s["seed"])) for s in scenarios) + "};",
           "static void acf_apply_seed(P_acf_rl_native_T *p,int scenario,int seed) {",
           "if(seed<0)return; double delta=seed-acf_bank_sensor_seeds[scenario];"]
    aux.extend(f"p->{name} += delta;" for name in seeds)
    aux.append("}\n")
    aux.append("static int acf_parameter_equal(const P_acf_rl_native_T *a,const P_acf_rl_native_T *b) {")
    for field in mapping:
        name,count=field["name"],int(field["count"])
        if count==1:aux.append(f"if(a->{name} != b->{name})return 0;")
        else:aux.append(f"for(int k=0;k<{count};k++)if(a->{name}[k] != b->{name}[k])return 0;")
    aux.append("return 1;}\n")
    (folder / "native_bank_aux.h").write_text("\n".join(aux))
    matroot = Path("/home/tommie/.local/opt/MATLAB/R2026b")
    recipe = Path(__file__).with_name("native_bridge.c")
    target = folder / "libacf_native.so"
    command = ["gcc", "-std=c11", "-O3", "-fPIC", "-shared", "-fopenmp", "-march=native", "-fno-fast-math", "-ffp-contract=off",
               "-I"+str(code), "-I"+str(folder), "-I"+str(matroot/"simulink/include"),
               "-I"+str(matroot/"extern/include"), str(recipe), str(folder/"native_parameter_bank.c"),
               str(code/"acf_rl_native_data.c"), str(code/"rtGetNaN.c"), str(code/"rt_nonfinite.c"),
               "-lm", "-pthread", "-o", str(target)]
    completed = subprocess.run(command, text=True, capture_output=True)
    (folder/"c_compile.log").write_text(completed.stdout+completed.stderr)
    completed.check_returncode()
    seal = {"emitted_c_sha256":_hash(emitted), "adapted_c_sha256":_hash(adapted_path),
            "bridge_sha256":_hash(recipe), "parameter_bank_sha256":_hash(folder/"native_parameter_bank.c"),
            "shared_library_sha256":_hash(target), "native_computation_recovered_exactly":True,
            "command":command, "seed_override":"Add chosen_seed - bank_sensor_seed to every emitted native sensor seed; baseline bank values unchanged when seed=None."}
    (folder/"bridge_build.json").write_text(json.dumps(seal,indent=2)+"\n")
    return target


class NativePlant:
    def __init__(self, build_dir: str | Path, *, rebuild=False, threads=1):
        self.folder = Path(build_dir).resolve()
        library = self.folder / "libacf_native.so"
        if rebuild or not library.exists():
            library = build_library(self.folder)
        self.lib = ct.CDLL(str(library))
        self.metadata = json.loads((self.folder/"metadata.json").read_text())
        overlay = json.loads((self.folder/"scenarios.json").read_text())
        candidates = [Path(self.metadata["configsFile"]).parent/"manifest.json",
                      self.folder.parent/"scenario_bank/manifest.json"]
        self.scenarios = overlay
        for manifest in candidates:
            if manifest.exists():
                capture = json.loads(manifest.read_text())
                original = capture["cases"]
                if capture.get("sourceModelSha256")!=self.metadata["sourceModelSha256"]:
                    continue
                expected=self.metadata.get("configsSha256")
                if expected and capture.get("configsSha256")!=expected:
                    continue
                if len(original)==len(overlay):
                    if any(a["scenario"]!=b["scenario"] or
                           ("identity" in b and a["identity"]!=b["identity"])
                           for a,b in zip(original,overlay,strict=True)):
                        continue
                    self.scenarios = [dict(a,**b) for a,b in zip(original,overlay,strict=True)]
                    break
        self.observation_names = self.metadata["observationNames"]
        self.lib.acf_create.argtypes = [ct.c_int,ct.c_int,ct.c_int];self.lib.acf_create.restype = _VOID
        self.lib.acf_reset.argtypes = [_VOID,ct.c_int,ct.c_int,ct.c_int,_PTR,_PTR,_PTR,_PTR]
        self.lib.acf_snapshot.argtypes = [_VOID,_PTR,_PTR,_PTR,_PTR]
        self.lib.acf_step.argtypes = [_VOID,ct.c_double,ct.c_double,_PTR,_PTR,_PTR,_PTR]
        self.lib.acf_step_batch.argtypes = [ct.POINTER(_VOID),ct.c_size_t,_PTR,_PTR,_PTR,_PTR,_PTR]
        self.lib.acf_step_repeat_batch.argtypes = [ct.POINTER(_VOID),ct.c_size_t,_PTR,ct.c_int,_PTR,_PTR,_PTR,_PTR]
        self.lib.acf_set_threads.argtypes = [ct.c_int]
        self.lib.acf_get_threads.argtypes = []
        self.lib.acf_destroy.argtypes = [_VOID]
        self.lib.acf_set_record.argtypes = [_VOID,_PTR,ct.c_size_t]
        self.lib.acf_record_count.argtypes = [_VOID];self.lib.acf_record_count.restype = ct.c_size_t
        if not self.lib.acf_bank_zero_matches_codegen():
            raise RuntimeError("Prepared bank case0 differs from Coder's emitted native parameters")
        self.set_threads(threads)

    def set_threads(self, threads):
        """Independent native contexts only; one OpenMP team per batch."""
        if self.lib.acf_set_threads(int(threads)):
            raise ValueError("threads must be 1 through 64")

    @property
    def threads(self):
        return int(self.lib.acf_get_threads())

    def create(self, scenario=0, seed=None, baseline=False, record=False):
        return NativeContext(self,scenario,seed,baseline,record)

    def step_batch(self, contexts, actions):
        obs,truth,comparator,metrics=self.step_batch_arrays(contexts,actions)
        results=[_result(obs[k],truth[k],comparator[k],metrics[k]) for k in range(len(contexts))]
        for context,result in zip(contexts,results,strict=True):context.result=result
        return results

    def step_batch_arrays(self, contexts, actions, *, repeat=1):
        """Raw native outputs, avoiding Python result objects in rollout loops.

        Context.result remains the previous object; use these returned arrays
        directly or snapshot() when leaving this fast path. No physics changes.
        Duplicate contexts are rejected because OpenMP owns each exactly once.
        """
        actions = np.ascontiguousarray(actions,dtype=np.float64)
        count = len(contexts)
        if actions.shape != (count,2):raise ValueError("actions must be [N,2] SI current/robot requests")
        obs=np.empty((count,18));truth=np.empty((count,27));comparator=np.empty((count,3));metrics=np.empty((count,24))
        handles=(_VOID*count)(*(c.handle for c in contexts))
        if int(repeat)!=repeat or repeat<1 or repeat>10000:
            raise ValueError("repeat must be an integer 1 through 10000")
        if repeat==1:
            status=self.lib.acf_step_batch(handles,count,_pointer(actions),_pointer(obs),_pointer(truth),_pointer(comparator),_pointer(metrics))
        else:
            status=self.lib.acf_step_repeat_batch(handles,count,_pointer(actions),int(repeat),_pointer(obs),_pointer(truth),_pointer(comparator),_pointer(metrics))
        if status<0:raise RuntimeError("Native batch rejected an invalid context")
        return obs,truth,comparator,metrics

    def step_repeat_batch_arrays(self, contexts, actions, repeat):
        """Hold each request over repeat native100us steps, return total interval.

        Scoring sums count every physical tick; extrema/failures include all
        boundaries. Native clocks, gates, queues and acquisition still run at
        their original rates. The actor observes only the final causal frame.
        """
        return self.step_batch_arrays(contexts,actions,repeat=repeat)


class NativeContext:
    def __init__(self,plant,scenario,seed,baseline,record):
        self.plant=plant;self.handle=plant.lib.acf_create(scenario,-1 if seed is None else int(seed),int(baseline))
        if not self.handle:raise RuntimeError("Native context could not initialize")
        self._record=None;self._record_enabled=bool(record);self.scenario=int(scenario)
        self.result=self.snapshot()
        if record:self._enable_record()

    @property
    def obs(self):return self.result.obs

    def _enable_record(self):
        duration=self.plant.scenarios[self.scenario].get("duration_s",4.0)
        capacity=int(round(duration/self.plant.metadata["step_s"]))+64
        self._record=np.empty((capacity,31),dtype=np.float64)
        self.plant.lib.acf_set_record(self.handle,_pointer(self._record),capacity)

    def reset(self,scenario=None,seed=None,baseline=False):
        if scenario is not None:self.scenario=int(scenario)
        obs,truth,comparator,metrics=_arrays()
        status=self.plant.lib.acf_reset(self.handle,self.scenario,-1 if seed is None else int(seed),int(baseline),
            _pointer(obs),_pointer(truth),_pointer(comparator),_pointer(metrics))
        if status<0:raise ValueError("Unknown native scenario")
        self.result=_result(obs,truth,comparator,metrics)
        if self._record_enabled:self._enable_record()
        return self.result

    def snapshot(self):
        obs,truth,comparator,metrics=_arrays()
        self.plant.lib.acf_snapshot(self.handle,_pointer(obs),_pointer(truth),_pointer(comparator),_pointer(metrics))
        return _result(obs,truth,comparator,metrics)

    def step(self,current_request_A,robot_correction_m_s):
        obs,truth,comparator,metrics=_arrays()
        status=self.plant.lib.acf_step(self.handle,float(current_request_A),float(robot_correction_m_s),
            _pointer(obs),_pointer(truth),_pointer(comparator),_pointer(metrics))
        if status<0:raise RuntimeError("Invalid native context")
        self.result=_result(obs,truth,comparator,metrics);return self.result

    def take_record(self,copy=True):
        if self._record is None:return {"time":np.empty(0),"inputs":np.empty((0,3)),"truth":np.empty((0,27))}
        count=int(self.plant.lib.acf_record_count(self.handle));rows=self._record[:count]
        if copy:rows=rows.copy()
        return {"time":rows[:,0],"inputs":rows[:,1:4],"truth":rows[:,4:31]}

    def close(self):
        if self.handle:self.plant.lib.acf_destroy(self.handle);self.handle=None

    def __del__(self):
        try:self.close()
        except Exception:pass


def _pointer(array):return array.ctypes.data_as(_PTR)
def _arrays():return np.empty(18),np.empty(27),np.empty(3),np.empty(24)
def _result(obs,truth,comparator,metrics):
    values=dict(zip(METRIC_NAMES,metrics.tolist(),strict=True))
    for key in ("native_samples","voltage_clip_samples","current_limit_samples","force_outside_samples","record_samples","physical_tick"):
        values[key]=int(values[key])
    for key in ("contact_loss","travel_stop","nonfinite","terminated"):values[key]=bool(values[key])
    return StepResult(obs,truth,comparator,values)


if __name__ == "__main__":
    import argparse
    parser=argparse.ArgumentParser();parser.add_argument("build_dir");args=parser.parse_args()
    plant=NativePlant(args.build_dir,rebuild=True)
    context=plant.create(0,baseline=True)
    print(json.dumps({"parameter_bank_matches_codegen":True,"initial":context.result.metrics,
                      "first_step":context.step(context.result.comparator[2],0).metrics},indent=2))
