package frc.robot.Subsystems.Subsystems;




// DO NOT USE THIS
// THIS IS A TEST TO SATISFY PERSTONAL PROBLEMS




public class UnitNumber {
    public double value;
    public Unit unit;
    public UnitNumber(double value, Unit unit)
    {
        this.value = value;
        this.unit = unit;
    }
    public enum Unit
    {
        //Rotational units
        EncoderUnit(new UnitNumber[]{}),
        FalconEncoderUnit(new UnitNumber[]{}),
        Rotation(new UnitNumber[]{new UnitNumber(2048, Unit.FalconEncoderUnit), new UnitNumber(1024, Unit.EncoderUnit)}),
        Degree(new UnitNumber[]{new UnitNumber(1/360,Unit.Rotation)}),
        Radian(new UnitNumber[]{new UnitNumber(180/Math.PI, Unit.Degree)}),
        ;

        public UnitNumber[] equities;
        Unit(UnitNumber[] equities) {this.equities = equities;}
    }
    public UnitNumber convert(Unit u)
    {
        for (UnitNumber e : this.unit.equities)
        {
            UnitNumber p = new UnitNumber(this.value * e.value,e.unit);
            if (p.unit == u)
            {
                return p;
            }
            else
            {
                p.convert(u);
            }
        }
        for (UnitNumber e : u.equities)
        {
            if (e.unit == this.unit)
            {
                return new UnitNumber(this.value / e.value,u);
            }
        }
        return null;
    }
}
