use std::time::SystemTime;
use std::time::UNIX_EPOCH;

fn main() {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards")
        .as_secs();
    
    // Export the current epoch time so the code can use it
    println!("cargo:rustc-env=BUILD_EPOCH={}", now);
    
    // Standard linker logic
    println!("cargo:rerun-if-changed=memory.x");
}
