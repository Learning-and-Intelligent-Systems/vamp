#pragma once

#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

// NOLINTBEGIN(*-magic-numbers)
namespace vamp::robots
{
struct Sphere
{
    static constexpr const char* name = "sphere";
    static constexpr std::size_t dimension = 3;
    static constexpr std::size_t n_spheres = 1;
    static constexpr float min_radius = 0.10000000149011612;
    static constexpr float max_radius = 0.10000000149011612;
    static constexpr std::size_t resolution = 32;

    static constexpr std::array<std::string_view, dimension> joint_names = {"x", "y", "theta"};
    static constexpr const char* end_effector = "base";

    using Configuration = FloatVector<dimension>;
    using ConfigurationArray = std::array<FloatT, dimension>;

    struct alignas(FloatVectorAlignment) ConfigurationBuffer
        : std::array<float, Configuration::num_scalars_rounded>
    {
    };

    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;

    template <std::size_t rake>
    struct Spheres
    {
        FloatVector<rake, n_spheres> x;
        FloatVector<rake, n_spheres> y;
        FloatVector<rake, n_spheres> z;
        FloatVector<rake, n_spheres> r;
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_m{
        5.0, 5.0, 6.283180236816406
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> s_a{
        -2.5, -2.5, -3.141590118408203
    };

    alignas(Configuration::S::Alignment) static constexpr std::array<float, dimension> d_m{
        0.20000000298023224, 0.20000000298023224, 0.15915507078170776
    };

    static inline void scale_configuration(Configuration& q) noexcept
    {
        q = q * Configuration(s_m) + Configuration(s_a);
    }

    static inline void descale_configuration(Configuration& q) noexcept
    {
        q = (q - Configuration(s_a)) * Configuration(d_m);
    }

    template <std::size_t rake>
    static inline void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept
    {
        q[0] = -2.5 + (q[0] * 5.0);
q[1] = -2.5 + (q[1] * 5.0);
q[2] = -3.141590118408203 + (q[2] * 6.283180236816406);

    }

    template <std::size_t rake>
    static inline void descale_configuration_block(ConfigurationBlock<rake> & q) noexcept
    {
        q[0] = 0.20000000298023224 * (q[0] - -2.5);
q[1] = 0.20000000298023224 * (q[1] - -2.5);
q[2] = 0.15915507078170776 * (q[2] - -3.141590118408203);

    }

    inline static auto space_measure() noexcept -> float
    {
        return 157.0795;
    }

    template <std::size_t rake>
    static inline void sphere_fk(const ConfigurationBlock<rake> &x, Spheres<rake> &out) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 4> y;

           y[0] = -0.228 + x[0];
   // dependent variables without operations
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.100000001490116;


        for (auto i = 0U; i < 1; ++i)
        {
            out.x[i] = y[i * 4 + 0];
            out.y[i] = y[i * 4 + 1];
            out.z[i] = y[i * 4 + 2];
            out.r[i] = y[i * 4 + 3];
        }
    }

    using Debug = std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    template <std::size_t rake>
        static inline auto fkcc_debug(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept -> Debug
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 8> y;

           y[0] = -0.228 + x[0];
   // variable duplicates: 1
   y[4] = y[0];
   // dependent variables without operations
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.100000001490116;
   y[5] = x[1];
   y[6] = 0.;
   y[7] = 0.100000001490116;


        Debug output;

        
        output.first.emplace_back(
            sphere_environment_get_collisions<decltype(x[0])>(
                environment,
                y[0],
                y[1],
                y[2],
                y[3]));
        

        

        return output;
    }

    template <std::size_t rake>
        static inline bool fkcc(
            const vamp::collision::Environment<FloatVector<rake>> &environment,
            const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 8> y;

           y[0] = -0.228 + x[0];
   // variable duplicates: 1
   y[4] = y[0];
   // dependent variables without operations
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.100000001490116;
   y[5] = x[1];
   y[6] = 0.;
   y[7] = 0.100000001490116;

        





//
// environment vs. robot collisions
//

// base
if (sphere_environment_in_collision(environment,
                                    y[4],
                                    y[5],
                                    y[6],
                                    y[7]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        return true;
    }

    template <std::size_t rake>
    static inline bool fkcc_attach(
        const vamp::collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &x) noexcept
    {
        std::array<FloatVector<rake, 1>, 0> v;
        std::array<FloatVector<rake, 1>, 20> y;

           y[0] = -0.228 + x[0];
   y[11] = cos(x[2]);
   y[12] = sin(x[2]);
   y[14] = - y[12];
   // variable duplicates: 3
   y[4] = y[0];
   y[8] = y[0];
   y[15] = y[11];
   // dependent variables without operations
   y[1] = x[1];
   y[2] = 0.;
   y[3] = 0.100000001490116;
   y[5] = x[1];
   y[6] = 0.;
   y[7] = 0.100000001490116;
   y[9] = x[1];
   y[10] = 0.;
   y[13] = 0.;
   y[16] = 0.;
   y[17] = 0.;
   y[18] = 0.;
   y[19] = 1.;

        





//
// environment vs. robot collisions
//

// base
if (sphere_environment_in_collision(environment,
                                    y[4],
                                    y[5],
                                    y[6],
                                    y[7]))
{
    
    
    if (sphere_environment_in_collision(environment,
                                        y[0],
                                        y[1],
                                        y[2],
                                        y[3]))
    {
        return false;
    }
    
}



//
// robot self-collisions
//





        // attaching at base
        set_attachment_pose(environment, to_isometry(&y[8]));

        //
        // attachment vs. environment collisions
        //
        if (attachment_environment_collision(environment))
        {
            return false;
        }

        //
        // attachment vs. robot collisions
        //

        

        return true;
    }

    static inline auto eefk(const std::array<float, 3> &x) noexcept -> Eigen::Isometry3f
    {
        std::array<float, 0> v;
        std::array<float, 12> y;

           y[0] = -0.228 + x[0];
   y[3] = cos(x[2]);
   y[4] = sin(x[2]);
   y[6] = - y[4];
   // variable duplicates: 1
   y[7] = y[3];
   // dependent variables without operations
   y[1] = x[1];
   y[2] = 0.;
   y[5] = 0.;
   y[8] = 0.;
   y[9] = 0.;
   y[10] = 0.;
   y[11] = 1.;


        return to_isometry(y.data());
    }
};
}

// NOLINTEND(*-magic-numbers)
