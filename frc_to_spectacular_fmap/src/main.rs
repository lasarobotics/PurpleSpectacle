use core::panic;
use std::{env, fs::File, io::Read, process, string};

use serde::{Deserialize, Serialize};

#[derive(serde::Deserialize, Debug)]
struct FrcMap {
    fiducials: Vec<FrcTag>,
}

#[derive(Serialize, Deserialize, Debug)]
struct FrcTag {
    id: u32,
    family: String,
    size: f64, //mm
    transform: [f64; 16],
}

#[derive(Serialize)]
struct SpectacularTag {
    id: u32,
    size: f64, //meters
    family: String,
    tagToWorld: Vec<Vec<f64>>,
}

fn main() {
    let args: Vec<String> = env::args().collect();

    // Check if the path argument was provided.
    if args.len() != 2 {
        eprintln!("Usage: {} <file_path>", args[0]);
        process::exit(1);
    }

    let mut file = File::open(&args[1]).expect("file not found");
    let mut content = String::new();
    let _ = file.read_to_string(&mut content);

    //println!("fmap file: {}", content);

    let json: FrcMap = serde_json::from_str(&mut content).expect("Invalid json");

    //println!("{:?}", json);

    let mut data: Vec<SpectacularTag> = Vec::new();
    for i in json.fiducials {
        data.push(SpectacularTag {
            id: i.id,
            family: match i.family.as_str() {
                "apriltag3_36h11_classic" => "tag36h11".to_string(),
                _ => panic!("Unsupported Apriltag family"),
            },
            size: (i.size / 1000.0),
            tagToWorld: i
                .transform
                .chunks(4)
                .map(|chunk| chunk.to_vec())
                .collect::<Vec<Vec<f64>>>(),
        });
    }

    println!(
        "{}",
        serde_json::to_string_pretty(&data).expect("idk what went wrong")
    );
}
