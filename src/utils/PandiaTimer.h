#pragma once

class PandiaTimer
{
private:
    std::chrono::high_resolution_clock::time_point m_old;
    PandiaTimer(const PandiaTimer &);
    PandiaTimer &operator=(const PandiaTimer &);

public:
    inline void reset() { m_old = std::chrono::high_resolution_clock::now(); }

    inline ~PandiaTimer() = default;

    inline PandiaTimer() { reset(); }

    inline double seconds() const
    {
        std::chrono::high_resolution_clock::time_point m_new =
            std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::duration<double>>(m_new -
                                                                         m_old)
            .count();
    }
};