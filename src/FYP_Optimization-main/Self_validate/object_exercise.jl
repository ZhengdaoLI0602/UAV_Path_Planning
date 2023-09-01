mutable struct Hero
    HP:: Float64
    ATK:: Float64
    name:: String

    function Hero(name::String, base_para::Float64)
        HP = base_para * 10
        ATK = base_para / 2
        new(HP, ATK, name)
    end
end

function attack_others(this_hero:: Hero, amount::Float64)
    this_hero.HP = this_hero.HP - amount
end

hero1 = Hero("A", 10)
hero2 = Hero("B", 5)

println("Hero1's HP is $(hero1.HP)")
println("Hero2's HP is $(hero2.HP)")
println("Hero1's ATK is $(hero1.ATK)")
println("Hero2's ATK is $(hero2.ATK)")

attack_others(hero1, 10.0)
attack_others(hero2, 1.0)

println("Hero1's HP is $(hero1.HP)")
println("Hero2's HP is $(hero2.HP)")