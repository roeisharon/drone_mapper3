// Fixture plugin: a valid .so that registers NOTHING (the loader must classify it as a load
// error — registration delta 0 — and roll back cleanly).

namespace TestAlgoNone_0_0 {

// A single exported symbol so the library is non-trivial; deliberately no REGISTER_* global.
extern "C" int test_algo_none_marker() {
    return 42;
}

} // namespace TestAlgoNone_0_0
