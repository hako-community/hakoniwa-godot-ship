using Godot;

namespace HakoniwaShip;

/// <summary>
/// SHP-23 Step 1: エントリシーンのルート。
/// 座標系: 箱庭/MuJoCo は ENU (x=East, y=North, z=Up)、Godot は Y-up 右手系。
/// 変換は ToGodot() に集約し、Step 2 以降の PDU 同期でも本関数のみを使う。
/// `-- --smoke` 付きで起動すると数フレーム描画後に HAKONIWA_GODOT_SHIP_BOOT_OK を出力して終了する。
/// </summary>
public partial class ShipViewerMain : Node3D
{
    private const int SmokeFrames = 10;

    private bool _smoke;
    private int _frames;
    private Label _hud;

    public override void _Ready()
    {
        _smoke = System.Array.IndexOf(OS.GetCmdlineUserArgs(), "--smoke") >= 0;
        _hud = GetNodeOrNull<Label>("Hud/Status");
        GD.Print($"hakoniwa-godot-ship: Godot {Engine.GetVersionInfo()["string"]}, renderer={RenderingServer.GetCurrentRenderingMethod()}");
    }

    public override void _Process(double delta)
    {
        _frames++;
        if (_hud != null)
        {
            _hud.Text = $"hakoniwa-godot-ship  |  FPS {Engine.GetFramesPerSecond():0}  |  PDU: not connected (Step 2)";
        }
        if (_smoke && _frames >= SmokeFrames)
        {
            GD.Print("HAKONIWA_GODOT_SHIP_BOOT_OK");
            GetTree().Quit(0);
        }
    }

    /// <summary>ENU [m] → Godot ワールド座標（East→+X, North→-Z, Up→+Y）。</summary>
    public static Vector3 ToGodot(double east, double north, double up)
    {
        return new Vector3((float)east, (float)up, (float)-north);
    }
}
