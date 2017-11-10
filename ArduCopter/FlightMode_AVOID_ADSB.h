class FlightMode_Avoid_ADSB : protected FlightMode_Guided {

public:

    FlightMode_Avoid_ADSB(Copter &copter) :
        Copter::FlightMode_Guided(copter)        { }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; }
    bool is_autopilot() const override { return true; }

    bool set_velocity(const Vector3f& velocity_neu);

protected:

    const char *name() const override { return "AVOID_ADSB"; }

private:

};
