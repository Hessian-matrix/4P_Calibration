use std::cmp::Ordering;

use ccl_calib::camera::rig::SharedView;

const MAX_SEED_VIEWS: usize = 32;

#[derive(Clone, Copy)]
struct Candidate {
    index: usize,
    support: usize,
    spread: f64,
    center: [f64; 2],
    extent: [f64; 2],
    time: f64,
}

impl Candidate {
    fn from_view(index: usize, view_count: usize, view: &SharedView<'_>) -> Option<Self> {
        let mut support = 0;
        let mut minimum = [f64::INFINITY; 2];
        let mut maximum = [f64::NEG_INFINITY; 2];

        for observation in view.observations {
            let width = f64::from(observation.image_size.width);
            let height = f64::from(observation.image_size.height);
            if width == 0.0 || height == 0.0 {
                continue;
            }
            for pixel in observation.points.pixels() {
                // A modest guard band retains legitimate edge detections without allowing a
                // wildly out-of-image point to dominate every diversity distance.
                let normalized = [
                    (pixel[0] / width).clamp(-2.0, 3.0),
                    (pixel[1] / height).clamp(-2.0, 3.0),
                ];
                for axis in 0..2 {
                    minimum[axis] = minimum[axis].min(normalized[axis]);
                    maximum[axis] = maximum[axis].max(normalized[axis]);
                }
                support += 1;
            }
        }
        if support == 0 {
            return None;
        }

        let extent = [maximum[0] - minimum[0], maximum[1] - minimum[1]];
        Some(Self {
            index,
            support,
            spread: extent[0] * extent[1],
            center: [
                (minimum[0] + maximum[0]) * 0.5,
                (minimum[1] + maximum[1]) * 0.5,
            ],
            extent,
            time: normalized_position(index, view_count),
        })
    }

    fn diversity_distance_squared(self, other: Self) -> f64 {
        let center_x = self.center[0] - other.center[0];
        let center_y = self.center[1] - other.center[1];
        let extent_x = self.extent[0] - other.extent[0];
        let extent_y = self.extent[1] - other.extent[1];
        let time = self.time - other.time;
        3.0 * (center_x * center_x + center_y * center_y)
            + extent_x * extent_x
            + extent_y * extent_y
            + 0.35 * time * time
    }
}

/// Chooses a small, strong, spatially and temporally varied intrinsic bootstrap set.
///
/// View order is the only timeline available at this layer. The returned indices retain their
/// original order; selection never mutates or narrows the later full calibration solve.
pub(super) fn select(views: &[SharedView<'_>]) -> Vec<usize> {
    let mut candidates = views
        .iter()
        .enumerate()
        .filter_map(|(index, view)| Candidate::from_view(index, views.len(), view))
        .collect::<Vec<_>>();
    let Some(max_support) = candidates.iter().map(|candidate| candidate.support).max() else {
        return Vec::new();
    };
    let max_spread = candidates
        .iter()
        .map(|candidate| candidate.spread)
        .fold(0.0, f64::max);

    // Relative gates work for sparse recordings while rejecting narrow two-tag frames whenever
    // the same recording contains materially stronger observations. Broad views get a little
    // latitude in point count because their image geometry is useful to the intrinsic seed.
    let support_floor = fraction_ceiling(max_support, 2, 5);
    let strong_support_floor = fraction_ceiling(max_support, 3, 5);
    candidates.retain(|candidate| {
        candidate.support >= support_floor
            && (candidate.support >= strong_support_floor
                || candidate.spread >= max_spread * 0.2)
    });

    if candidates.len() <= MAX_SEED_VIEWS {
        return candidates
            .into_iter()
            .map(|candidate| candidate.index)
            .collect();
    }

    let first = candidates
        .iter()
        .enumerate()
        .max_by(|(_, left), (_, right)| quality_order(left, right))
        .map_or(0, |(position, _)| position);
    let mut selected = Vec::with_capacity(MAX_SEED_VIEWS);
    selected.push(candidates.swap_remove(first));

    while selected.len() < MAX_SEED_VIEWS {
        let mut best = None;
        for (position, candidate) in candidates.iter().enumerate() {
            let distance = selected
                .iter()
                .map(|chosen| candidate.diversity_distance_squared(*chosen))
                .min_by(f64::total_cmp)
                .unwrap_or(0.0);
            if best.is_none_or(|(best_position, best_distance)| {
                distance.total_cmp(&best_distance) == Ordering::Greater
                    || (distance.total_cmp(&best_distance) == Ordering::Equal
                        && quality_order(candidate, &candidates[best_position])
                            == Ordering::Greater)
            }) {
                best = Some((position, distance));
            }
        }
        let position = best.map_or(0, |(position, _)| position);
        selected.push(candidates.swap_remove(position));
    }

    selected.sort_unstable_by_key(|candidate| candidate.index);
    selected
        .into_iter()
        .map(|candidate| candidate.index)
        .collect()
}

fn quality_order(left: &Candidate, right: &Candidate) -> Ordering {
    left.support
        .cmp(&right.support)
        .then_with(|| left.spread.total_cmp(&right.spread))
        .then_with(|| right.index.cmp(&left.index))
}

fn fraction_ceiling(value: usize, numerator: usize, denominator: usize) -> usize {
    let quotient = value / denominator;
    let remainder = value % denominator;
    quotient * numerator + (remainder * numerator).div_ceil(denominator)
}

fn normalized_position(index: usize, count: usize) -> f64 {
    let last = u32::try_from(count.saturating_sub(1)).unwrap_or(u32::MAX);
    if last == 0 {
        return 0.0;
    }
    let index = u32::try_from(index).unwrap_or(u32::MAX);
    f64::from(index) / f64::from(last)
}

#[cfg(test)]
mod tests {
    use ccl_calib::camera::{
        CalibrationImageSize, CameraObservation, PointCorrespondences, TargetGeometry,
    };

    use super::select;
    use ccl_calib::camera::rig::SharedView;

    const IMAGE_SIZE: CalibrationImageSize = CalibrationImageSize {
        width: 1920,
        height: 1080,
    };

    fn pixel_grid(center: [f64; 2], extent: [f64; 2], count: usize) -> Vec<[f64; 2]> {
        let columns = 4;
        let rows = count / columns;
        (0..rows)
            .flat_map(|row| {
                (0..columns).map(move |column| {
                    let x = f64::from(u32::try_from(column).unwrap())
                        / f64::from(u32::try_from(columns - 1).unwrap());
                    let y = f64::from(u32::try_from(row).unwrap())
                        / f64::from(u32::try_from(rows.saturating_sub(1).max(1)).unwrap());
                    [
                        center[0] + (x - 0.5) * extent[0],
                        center[1] + (y - 0.5) * extent[1],
                    ]
                })
            })
            .collect()
    }

    fn observations<'a>(
        target: &'a TargetGeometry,
        pixels: &'a [Vec<[f64; 2]>],
        indices: &'a [Vec<usize>],
    ) -> Vec<[CameraObservation<'a>; 1]> {
        pixels
            .iter()
            .zip(indices)
            .map(|(pixels, indices)| {
                [CameraObservation {
                    camera_index: 0,
                    image_size: IMAGE_SIZE,
                    points: PointCorrespondences::from_indexed_points(target, indices, pixels)
                        .unwrap(),
                }]
            })
            .collect()
    }

    fn target() -> TargetGeometry {
        TargetGeometry::new(
            (0_u32..64)
                .map(|index| [f64::from(index % 8), f64::from(index / 8), 0.0])
                .collect(),
        )
        .unwrap()
    }

    #[test]
    fn sparse_narrow_views_are_suppressed_when_stronger_views_exist() {
        let target = target();
        let pixels = (0..8)
            .map(|index| {
                if index == 2 || index == 5 {
                    pixel_grid([1840.0, 500.0], [70.0, 45.0], 8)
                } else {
                    pixel_grid([800.0 + f64::from(index) * 25.0, 540.0], [850.0, 600.0], 24)
                }
            })
            .collect::<Vec<_>>();
        let indices = pixels
            .iter()
            .map(|pixels| (0..pixels.len()).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let observations = observations(&target, &pixels, &indices);
        let views = observations
            .iter()
            .map(|observations| SharedView { observations })
            .collect::<Vec<_>>();

        assert_eq!(select(&views), vec![0, 1, 3, 4, 6, 7]);
    }

    #[test]
    fn uniformly_sparse_recording_uses_relative_support_thresholds() {
        let target = target();
        let pixels = (0..10)
            .map(|index| {
                pixel_grid(
                    [300.0 + f64::from(index) * 130.0, 540.0],
                    [150.0, 100.0],
                    8,
                )
            })
            .collect::<Vec<_>>();
        let indices = pixels
            .iter()
            .map(|pixels| (0..pixels.len()).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let observations = observations(&target, &pixels, &indices);
        let views = observations
            .iter()
            .map(|observations| SharedView { observations })
            .collect::<Vec<_>>();

        assert_eq!(select(&views), (0..views.len()).collect::<Vec<_>>());
    }

    #[test]
    fn capped_seed_is_temporally_and_geometrically_diverse() {
        let target = target();
        let centers = [[300.0, 300.0], [960.0, 760.0], [1620.0, 360.0]];
        let pixels = (0..96)
            .map(|index| pixel_grid(centers[index % centers.len()], [500.0, 360.0], 24))
            .collect::<Vec<_>>();
        let indices = pixels
            .iter()
            .map(|pixels| (0..pixels.len()).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let observations = observations(&target, &pixels, &indices);
        let views = observations
            .iter()
            .map(|observations| SharedView { observations })
            .collect::<Vec<_>>();

        let selected = select(&views);
        assert_eq!(selected.len(), 32);
        assert!(selected.windows(2).all(|pair| pair[0] < pair[1]));
        assert_eq!(selected, select(&views));
        assert!(selected.contains(&0));
        assert!(selected.contains(&95));
        assert!((0..4).all(|quarter| selected.iter().any(|index| index / 24 == quarter)));
        assert!((0..centers.len()).all(|geometry| selected.iter().any(|index| index % 3 == geometry)));
    }
}
