#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <vamp/collision/shapes.hh>
#include <vector>

namespace vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(const Eigen::Transform<DataT, 3, Eigen::Isometry> &tf) noexcept : tf(std::move(tf))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Eigen::Transform<float, 3, Eigen::Isometry> &tf) noexcept
          : Attachment(tf.cast<DataT>())
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept : Attachment(o.tf)
        {
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }
        }

        std::vector<Sphere<DataT>> spheres;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        // One sphere containing every sphere of the attachment, in its frame, and posed:
        // the checks screen the attachment through it before testing its spheres
        // (validity.hh), as the robot's own spheres are screened through their links'
        // bounding spheres. Rebuilt when the number of spheres changes.
        mutable std::size_t bound_n = 0;
        mutable Sphere<DataT> bound;
        mutable Sphere<DataT> posed_bound;

        inline void pose(const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) const noexcept
        {
            const auto &n_tf = p_tf * tf;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                Eigen::Matrix<DataT, 3, 1> sp(s.x, s.y, s.z);
                auto tfs = n_tf * sp;
                posed_spheres[i] = Sphere<DataT>(tfs[0], tfs[1], tfs[2], s.r);
            }

            if (spheres.size() > 1)
            {
                if (bound_n != spheres.size())
                {
                    make_bound();
                }
                Eigen::Matrix<DataT, 3, 1> bp(bound.x, bound.y, bound.z);
                auto tfb = n_tf * bp;
                posed_bound = Sphere<DataT>(tfb[0], tfb[1], tfb[2], bound.r);
            }
        }

      private:
        inline static auto lane(const DataT &v) noexcept -> float
        {
            if constexpr (std::is_same_v<DataT, float>)
            {
                return v;
            }
            else
            {
                return v[{0, 0}];
            }
        }

        // Centre of the spheres' bounding box; radius reaching the farthest sphere's surface,
        // rounded up by 1e-5 m. The spheres are the same in every lane.
        inline void make_bound() const noexcept
        {
            float lo[3], hi[3];
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                const float c[3] = {lane(s.x), lane(s.y), lane(s.z)};
                const float r = lane(s.r);
                for (auto k = 0U; k < 3; ++k)
                {
                    lo[k] = (i == 0) ? c[k] - r : std::min(lo[k], c[k] - r);
                    hi[k] = (i == 0) ? c[k] + r : std::max(hi[k], c[k] + r);
                }
            }
            const float m[3] = {0.5F * (lo[0] + hi[0]), 0.5F * (lo[1] + hi[1]), 0.5F * (lo[2] + hi[2])};
            float radius = 0.0F;
            for (const auto &s : spheres)
            {
                const float dx = lane(s.x) - m[0], dy = lane(s.y) - m[1], dz = lane(s.z) - m[2];
                radius = std::max(radius, std::sqrt(dx * dx + dy * dy + dz * dz) + lane(s.r));
            }
            bound = Sphere<DataT>(DataT(m[0]), DataT(m[1]), DataT(m[2]), DataT(radius + 1e-5F));
            bound_n = spheres.size();
        }
    };
}  // namespace vamp::collision
