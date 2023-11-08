#include <SFML/Graphics.hpp>
#include <Box2D/Box2D.h>
#include <Box2D/b2_time_of_impact.h>
#include <iostream>

const float TamanoEscala = 30.0f;

int main() {

    sf::RenderWindow window(sf::VideoMode(800, 600), "Bolita");
    window.setFramerateLimit(60);

    // Configuración
    b2Vec2 Gravedad(0.0f, -5.0f);
    b2World Mundo(Gravedad);

    // Crear suelo
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, 0.0f);
    b2Body* groundBody = Mundo.CreateBody(&groundBodyDef);

    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0f, 1.0f);

    b2FixtureDef groundFixtureDef;
    groundFixtureDef.shape = &groundBox;
    groundFixtureDef.density = 0.0f;
    groundBody->CreateFixture(&groundFixtureDef);

    // Creacion de la bola
    b2BodyDef CuaerpoBolita;
    CuaerpoBolita.type = b2_dynamicBody;
    CuaerpoBolita.position.Set(0.0f, 10.0f);
    b2Body* BolitaBody = Mundo.CreateBody(&CuaerpoBolita);

    b2PolygonShape BolitaShape;
    BolitaShape.SetAsBox(1.0f, 1.0f);

    b2FixtureDef projectileFixtureDef;
    projectileFixtureDef.shape = &BolitaShape;
    projectileFixtureDef.density = 1.0f;
    projectileFixtureDef.friction = 0.3f;
    BolitaBody->CreateFixture(&projectileFixtureDef);

    sf::CircleShape CirculoBolita(10.0f);
    CirculoBolita.setFillColor(sf::Color::Red);

    BolitaBody->SetLinearVelocity(b2Vec2(10.0f, 0.0f));

    b2DistanceInput distInput;
    distInput.proxyA.Set(&BolitaShape, 0);
    distInput.proxyB.Set(&groundBox, 0);
    distInput.transformA = BolitaBody->GetTransform();
    distInput.transformB = groundBody->GetTransform();
    distInput.useRadii = true;

    b2DistanceOutput distOutput;
    b2SimplexCache cache;

    cache.count = 0;

    b2Distance(&distOutput, &cache, &distInput);

    printf("Distance between bodies: %4.2f m\n", distOutput.distance);

    b2Sweep sweepA;
    sweepA.c0 = BolitaBody->GetPosition();
    sweepA.a0 = BolitaBody->GetAngle();
    sweepA.c = BolitaBody->GetWorldCenter() + BolitaBody->GetLinearVelocity();
    sweepA.a = sweepA.a0;
    sweepA.localCenter.SetZero();

    b2Sweep sweepB;
    sweepB.c0 = groundBody->GetPosition();
    sweepB.a0 = groundBody->GetAngle();
    sweepB.c = groundBody->GetWorldCenter() + groundBody->GetLinearVelocity();
    sweepB.a = sweepB.a0;
    sweepB.localCenter.SetZero();

    float tMax = 10.0f;

    b2TOIInput toiInput;
    toiInput.proxyA.Set(&BolitaShape, 0);
    toiInput.proxyB.Set(&groundBox, 0);
    toiInput.sweepA = sweepA;
    toiInput.sweepB = sweepB;
    toiInput.tMax = tMax;

    b2TOIOutput toiOutput{};
    b2TimeOfImpact(&toiOutput, &toiInput);

    float timeOfImpact = toiOutput.t;
    printf("Time of impact: %.2f s\n", timeOfImpact);

    //
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        Mundo.Step(1.0f / 60.0f, 6, 2);

        b2Vec2 position = BolitaBody->GetPosition();
        CirculoBolita.setPosition(position.x * TamanoEscala, window.getSize().y - position.y * TamanoEscala);

        window.clear(sf::Color::White);

        window.draw(CirculoBolita);

        window.display();
    }

    return 0;
}