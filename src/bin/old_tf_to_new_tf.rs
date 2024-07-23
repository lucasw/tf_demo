use std::collections::HashMap;
use std::time::SystemTime;

use tf_roslibrust::{
    TfListener,
    tf_util,
    transforms::tf2_msgs::TFMessage,
};

// roslibrust_codegen_macro::find_and_generate_ros_messages!();

/// Take in a source and destination frame argument
/// and repeatedly print the transform between them if any

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    use roslibrust::ros1::NodeHandle;
    use roslibrust::ros1::Publisher;

    // need to have leading slash on node name and topic to function properly
    // so figure out namespace then prefix it to name and topics
    let mut ns = String::from("");

    // string parameters
    let mut param_str = HashMap::<String, String>::new();
    param_str.insert("lookup_parent".to_string(), "map".to_string());
    param_str.insert("lookup_child".to_string(), "base_link".to_string());
    param_str.insert("broadcast_parent".to_string(), "map".to_string());
    param_str.insert("broadcast_child".to_string(), "tmp".to_string());

    let args = std::env::args();
    let mut args2 = Vec::new();
    {
        for arg in args {
            let key_val: Vec<&str> = arg.split(":=").collect();
            if key_val.len() != 2 {
                args2.push(arg);
                continue;
            }

            let (mut key, val) = (key_val[0].to_string(), key_val[1].to_string());
            if !key.starts_with("_") {
                println!("unused arg pair {key}:={val}- need to prefix name with underscore");
                continue;
            }
            key.replace_range(0..1, "");

            // get namespace, __ns -> _ns because of above replacement
            if key == "_ns" {
               ns = val;
            } else if param_str.contains_key(&key) {
                param_str.insert(key, val);  // .to_string();
            }
        }
    }
    println!("{args2:?}");
    println!("{param_str:?}");

    let full_node_name = &format!("/{ns}/old_tf_to_new_tf").replace("//", "/");
    // println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await?;

    // TODO(lucasw) could use tf_broadcaster but it doesn't really add much
    let tf_pub: Publisher<TFMessage> = nh.advertise("/tf", 100).await?;

    let mut listener = TfListener::new(&nh).await;

    let update_period = tokio::time::Duration::from_millis(50);
    let mut next_update = SystemTime::now();

    loop {
        // sleep for remaining if nothing else interrupt select, or sleep for 0 seconds
        // if already past scheduled update
        let remaining = {
            let time_now = SystemTime::now();
            let remaining;
            if time_now > next_update {
                remaining = tokio::time::Duration::from_secs(0);
            } else {
                remaining = next_update.duration_since(time_now)?;
            }
            remaining
        };

        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("ctrl-c exiting");
                break;
            }
            // TODO(lucasw) move this into listener
            rv = listener._dynamic_subscriber.next() => {
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf(tfm).await;
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            }
            rv = listener._static_subscriber.next() => {
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf_static(tfm).await;
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            }
            _ = tokio::time::sleep(remaining) => {
                next_update += update_period;
                // println!("update {remaining:?}");
                // let lookup_stamp = tf_util::stamp_now();
                // TODO(lucasw) maybe just have a lookup_transform_recent function
                // TODO(lucasw) swapping position of frame 1 2 to match tf2_tools echo.py
                let res = listener.lookup_transform(
                    &param_str["lookup_parent"],
                    &param_str["lookup_child"],
                    None);
                let stamp_now = tf_util::stamp_now();
                match res {
                    Ok(tf) => {
                        // TODO(lucasw) if tf has the same stamp as the last update don't publish
                        // it
                        let mut tf_out = tf.clone();
                        tf_out.header.frame_id = param_str["broadcast_parent"].clone();
                        tf_out.child_frame_id = param_str["broadcast_child"].clone();
                        let tfm = TFMessage {
                            transforms: vec![tf_out],
                        };
                        tf_pub.publish(&tfm).await?;
                    },
                    Err(err) => { println!("{stamp_now:?} {err:?}"); },
                }
            }
        }  // tokio select loop
    }

    Ok(())
}
