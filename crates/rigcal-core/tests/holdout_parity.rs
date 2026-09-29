//! holdout / 代表性选择的对拍（确定性算法，要求**逐个下标一致**）。
//!
//! 夹具为 24 条确定性合成观测的桶键、四维特征、代表性选择（12/7/1/0 四种上限）与
//! 0.25 划分的 train/holdout 成员。比较用「首点坐标签名」而不是下标，避免两边索引语义漂移。

use rigcal_core::estimator::Observation;
use rigcal_core::holdout::{
    observation_bucket, observation_feature_vector, representative_indices, split_train_holdout,
};
use serde_json::Value;

const FIXTURE: &str = include_str!("fixtures/holdout_expected.json");

fn fixture() -> (Value, Vec<Observation>, (u32, u32)) {
    let value: Value = serde_json::from_str(FIXTURE).expect("fixture");
    let image_size = (
        value["image_size"][0].as_u64().unwrap() as u32,
        value["image_size"][1].as_u64().unwrap() as u32,
    );
    let observations = value["observations"]
        .as_array()
        .expect("observations")
        .iter()
        .map(|points| {
            let image_points = points
                .as_array()
                .expect("points")
                .iter()
                .map(|point| {
                    let point = point.as_array().expect("point");
                    [point[0].as_f64().unwrap(), point[1].as_f64().unwrap()]
                })
                .collect::<Vec<[f64; 2]>>();
            let object_points = vec![[0.0, 0.0, 0.0]; image_points.len()];
            Observation {
                object_points,
                image_points,
            }
        })
        .collect();
    (value, observations, image_size)
}

fn signature(observation: &Observation) -> [f64; 2] {
    observation.image_points[0]
}

#[test]
fn buckets_features_and_selection_match_python() {
    let (value, observations, image_size) = fixture();
    for (index, observation) in observations.iter().enumerate() {
        let bucket = observation_bucket(observation, image_size);
        let expected = &value["buckets"][index];
        assert_eq!(
            [bucket.0, bucket.1, bucket.2, bucket.3,],
            [
                expected[0].as_i64().unwrap() as i32,
                expected[1].as_i64().unwrap() as i32,
                expected[2].as_i64().unwrap() as i32,
                expected[3].as_i64().unwrap() as i32,
            ],
            "observation {index} bucket"
        );
        let feature = observation_feature_vector(observation, image_size);
        let expected = value["features"][index].as_array().expect("features");
        for axis in 0..4 {
            let want = expected[axis].as_f64().expect("value");
            assert!(
                (feature[axis] - want).abs() < 1e-12,
                "observation {index} feature[{axis}]: {} vs python {want}",
                feature[axis]
            );
        }
    }

    for (key, max_count) in [("12", 12usize), ("7", 7), ("1", 1), ("0", 0)] {
        let selected = representative_indices(&observations, max_count, image_size);
        let expected: Vec<[f64; 2]> = value["representative"][key]
            .as_array()
            .expect("representative")
            .iter()
            .map(|pair| [pair[0].as_f64().unwrap(), pair[1].as_f64().unwrap()])
            .collect();
        let actual: Vec<[f64; 2]> = selected
            .iter()
            .map(|index| signature(&observations[*index]))
            .collect();
        assert_eq!(
            actual, expected,
            "representative(max_count={max_count}) 选择必须逐个一致"
        );
    }

    let (train, holdout) = split_train_holdout(&observations, 0.25, image_size).expect("split");
    for (name, indices) in [("train", &train), ("holdout", &holdout)] {
        let actual: Vec<[f64; 2]> = indices
            .iter()
            .map(|index| signature(&observations[*index]))
            .collect();
        let expected: Vec<[f64; 2]> = value["split_0_25"][name]
            .as_array()
            .expect(name)
            .iter()
            .map(|pair| [pair[0].as_f64().unwrap(), pair[1].as_f64().unwrap()])
            .collect();
        assert_eq!(actual, expected, "{name} 成员必须逐个一致");
    }
    assert_eq!(train.len() + holdout.len(), observations.len());
    println!(
        "holdout 划分：{} 条观测 → train {} / holdout {}（与 Python 逐个下标一致）",
        observations.len(),
        train.len(),
        holdout.len()
    );
}
