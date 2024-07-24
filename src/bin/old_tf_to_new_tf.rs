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

    // TODO(lucasw) this update period has a big effect on cpu usage
    let mut update_interval = tokio::time::interval(tokio::time::Duration::from_millis(50));

    let mut last_published_time = None;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("ctrl-c exiting");
                break;
            }
            // TODO(lucasw) move this into listener
            rv = listener._dynamic_subscriber.next() => {
                // let t0 = tf_util::duration_now();
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf(tfm).await;
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
                // let t1 = tf_util::duration_now();
                // println!("{:?}", t1 - t0);  // this takes about 30 us
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
            _ = update_interval.tick() => {
                let t0 = tf_util::duration_now();
                // let lookup_stamp = tf_util::stamp_now();
                // TODO(lucasw) maybe just have a lookup_transform_recent function
                // TODO(lucasw) swapping position of frame 1 2 to match tf2_tools echo.py
                let res = listener.lookup_transform(
                    &param_str["lookup_parent"],
                    &param_str["lookup_child"],
                    None);
                let t1 = tf_util::duration_now();
                match res {
                    Ok(tf) => {
                        // TODO(lucasw) if tf has the same stamp as the last update don't publish
                        // it
                        let mut tf_out = tf.clone();
                        let tf_out_time = tf_util::stamp_to_duration(tf_out.header.stamp.clone());
                        if last_published_time.is_none() || (tf_out_time > last_published_time.unwrap()) {
                            tf_out.header.frame_id = param_str["broadcast_parent"].clone();
                            tf_out.child_frame_id = param_str["broadcast_child"].clone();
                            let tfm = TFMessage {
                                transforms: vec![tf_out],
                            };
                            tf_pub.publish(&tfm).await?;
                            last_published_time = Some(tf_out_time);
                        }
                    },
                    Err(err) => { println!("{t1:?} {err:?}"); },
                }
                let t2 = tf_util::duration_now();
                println!("lookup: {:?}, publish: {:?}", t1 - t0, t2 - t1); // the lookup takes about 220 us
            }
        }  // tokio select loop
    }

    Ok(())
}
